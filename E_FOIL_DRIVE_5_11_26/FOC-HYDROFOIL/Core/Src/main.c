/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   eFoil traction drive: sensorless FOC on STM32G431RB using a
 *          first-order sliding mode observer with equivalent-control
 *          extraction, CORDIC arctangent demodulation, and an angle
 *          tracking observer for speed.
 *
 * MACHINE AND POWER STAGE
 *   Flipsky 65161, 100 KV, p = 3 pole pairs (6 poles, 18 slots).
 *   Rs = 0.06 ohm (four-wire Kelvin, disconnected motor),
 *   Ls = 40 uH per phase (80 uH line-line / 2), lambda_F = 0.020130 Wb.
 *   Bus 50.4 V (12S), 20 kHz centre-aligned PWM on TIM1, dead time 1.98 us.
 *   Mitsubishi FM600TU-3A six-pack, 3x UCC27714, 3.01 ohm gate resistors.
 *   DC link: one CDE 944U221K801ACM plus ceramic terminal snubber, valid
 *   only inside the duty and RMS bounds enforced in cap_limit_update().
 *   Sensing: ACS72981-200B3 on phases A and B (i_c reconstructed), LEM
 *   HASS 300-S on the DC link, 180k/10k bus divider on PA4, module NTC on
 *   PB14.
 *
 * ===========================================================================
 * 1. MACHINE MODEL AND CLARKE CONVENTION
 * ===========================================================================
 * Amplitude-invariant Clarke (Mohan, Advanced Electric Drives, ch. 3):
 *     i_alpha = i_a,   i_beta = (i_a + 2 i_b)/sqrt(3)
 * so that for a balanced set of peak I, |i_alpha + j i_beta| = I. The torque
 * expression therefore carries the (3/2) factor explicitly:
 *     T_e = (3/2) p lambda_F i_q
 * State this convention in the report; the (3/2) is a consequence of the
 * transform scaling, not an independent constant.
 *
 * ===========================================================================
 * 5. TRACE RECORDER (debug only)
 * ===========================================================================
 * A ring buffer of TRACE_DEPTH x TRACE_CH_N int16 samples, written from the
 * FOC ISR and dumped as ASCII CSV over USART2 (PA2/PA3), which the NUCLEO
 * board wires to the ST-LINK virtual COM port. NOTHING in the .ioc changes:
 * USART2 is brought up entirely from user code in trace_uart_init(), and a
 * failure there disables the recorder rather than calling Error_Handler,
 * because a debug port must never be able to stop the drive.
 *
 * WHY A BURST RECORDER AND NOT A STREAM. Nine channels of float at 20 kHz
 * is 720 kB/s, which no UART here will carry. The record length is set by
 * RAM: 896 x 9 x int16 = 16128 bytes of the 32 kB on a G431RB. With
 * TRACE_DECIM that buys 45 ms at the full 20 kHz, 0.90 s at 1 kHz, or
 * 8.96 s at 100 Hz.
 *
 * WHAT IT CANNOT DO. There is no position or speed sensor on this machine,
 * so theta_ol is a COMMAND, not ground truth: the rotor lags it by the load
 * angle. theta_est against theta_ol is a convergence plot, not an accuracy
 * plot. An accuracy figure needs one index pulse per mechanical revolution
 * from an external sensor; say so in the report rather than implying the
 * forced angle is truth.
 *
 * ===========================================================================
 * TELEMETRY PROTOCOL (FIXED, the Pico rejects any other field count)
 *   STATUS "<L>,<st><fl>,<t10>,<rpm>,<idc>,<vb10>,<ia>,<ib>,<ic>\n"
 *          NINE fields, lead 'S' normally and 'L' in lab mode.
 *   DEBUG  "D,<id10>,<iq10>,<vd100>,<vq100>,<vmax100>\n"
 *          SIX fields, half the status rate, offset half a period.
 *   Sentinels: t10 = 9999 NTC faulted, vb10 < 0 bus sense faulted,
 *   idc = -9999 HASS never calibrated.
 *   The rpm field is a WINDOW AVERAGE over the same 100 ms the RMS currents
 *   use, not an instantaneous sample. An instantaneous sample of a signal
 *   with a 124 Hz bandwidth, taken at 10 Hz, is a random snapshot and was
 *   the second half of the unstable RPM display.
 *   The trace dump on USART2 is a SEPARATE port and does not touch this.
 *
 * PIN MAP
 *   PA8  TIM1_CH1  A high    PB13 TIM1_CH1N  A low
 *   PA9  TIM1_CH2  B high    PA12 TIM1_CH2N  B low
 *   PA10 TIM1_CH3  C high    PB15 TIM1_CH3N  C low
 *   PC10 GATE_EN (UCC27714 EN, all three, active high)
 *   PB6  USART1_TX -> HM-10 RX     PB7 USART1_RX <- HM-10 TX
 *   PA2  USART2_TX -> ST-LINK VCP (trace dump, user-code init, AF7)
 *   PA3  USART2_RX <- ST-LINK VCP (unused, configured for completeness)
 *   PA5  LD2 green LED
 *   PB14 (NTC) sits between PB13 and PB15 on the die and on the header, both
 *   switching at 20 kHz once MOE is set; the 100 nF at the pin is required.
 *   UCC27714 EN is pulled to VDD through 200 k, so GATE_EN is not a
 *   fail-safe interlock on its own. Safety between reset and MX_GPIO_Init
 *   comes from the 400 k internal pull-downs on HI/LI plus OSSI = 0 leaving
 *   the TIM1 pins Hi-Z while MOE is clear. Do NOT tie EN to ground.
 *
 * CUBEMX .IOC REQUIREMENTS (cannot be guarded from user code)
 *   1. ADC1 and ADC2 enabled, PA0/PA1/PA4/PB0/PB14 analog
 *   2. NVIC "ADC1 and ADC2 global interrupt" ENABLED
 *   3. TIM1 CH1-3 on PA8/PA9/PA10, CH1N/CH2N/CH3N on PB13/PA12/PB15
 *   4. USART1 RX and TX DMA (or the USART1 global interrupt as fallback)
 *   No regular-group channel is needed. The CORDIC clock is enabled in user
 *   code and needs no .ioc entry, and so is USART2 for the trace dump.

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    FOC_OFFSET_CAL = 0,   /* boot only: HASS zero + ACS check, gates off    */
    FOC_IDLE,             /* gates off, waiting for throttle                */
    FOC_ALIGN,            /* d-axis current ramp, rotor locks to theta = 0  */
    FOC_OPEN_LOOP,        /* forced-angle I-f ramp, observer converging     */
    FOC_CLOSED_LOOP,      /* theta from the SMO, throttle -> iq             */
    FOC_FAULT             /* latched fault                                  */
} foc_state_t;

/* Trace recorder, section 5 of the header block */
typedef enum {
    TRACE_OFF = 0,        /* not recording, nothing to dump                 */
    TRACE_RUN,            /* filling the ring                               */
    TRACE_FULL            /* frozen, waiting to be dumped                   */
} trace_state_t;

typedef struct {
    int32_t     k;        /* engineering units -> int16 counts. INTEGER on
                             purpose: the #scale header line prints this
                             value with %d, so a fractional scale here would
                             silently desynchronise the PC-side units from
                             the ones actually applied in trace_put().      */
    uint8_t     avg;      /* 1 = box average over the decimation window     */
    const char *name;     /* CSV column heading                             */
} trace_ch_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 1 = bench, 0 = water. Compile-time, so every branch folds away. */
#define LAB_TEST_MODE   1

#if LAB_TEST_MODE
#warning "LAB_TEST_MODE = 1: overtemperature and DC link trips are DISABLED."
#endif

/* ---- Timing --------------------------------------------------------------*/
#define F_PWM_HZ        20000.0f
#define TS              (1.0f / F_PWM_HZ)          /* 50 us                  */
#define PWM_ARR         4250                       /* 170 MHz / (2*4250)     */

/* Dead time. DTG = 0xCA decodes DTG[7:5] = 110, so
 *   t_dead = (32 + DTG[4:0]) * 8 * tDTS = 42 * 8 * 5.882 ns = 1.98 us,
 * against a requirement of td(off) 500 ns + tf 400 ns + 20 ns matching =
 * 920 ns from the FM600TU-3A dynamic table at RG = 4.2 ohm. 2.1x margin is
 * deliberate on a 300 A module with no desat protection. It costs 3.96
 * percent of the period, i.e. 2.0 V per phase at 50.4 V, which section 12
 * undoes. If shortened, move DEADTIME_S, DUTY_MAX and VMAX_FRAC together. */
#define DEADTIME_DTG    0xCA
#define DEADTIME_S      1.98e-6f

/* Duty limits, SET BY THE BOOTSTRAP, not by the modulator. Low-side
 * conduction per period is (1-d)*Ts - t_dead; each high-side turn-on removes
 * Qg/C_BOOT = 1950 nC / 4.7 uF = 0.415 V from HB-HS, and R_BOOT*C_BOOT =
 * 10.3 us. At d = 0.95 the recovery window is 4.9 percent and HB-HS settles
 * at 5.64 V, below the 8.05 V UVLO maximum. At d = 0.92 it settles at
 * 11.76 V. Raising DUTY_MAX needs a bigger C_BOOT, not a firmware edit. */
#define DUTY_MIN        0.08f // was 0.08f
#define DUTY_MAX        0.92f // was 0.92f

/* ---- Machine -------------------------------------------------------------
 * Ls matters more than Rs for the observer: in (1) the w*Ls*i term is
 * perpendicular to the current, so an Ls error is an ANGLE error, while an
 * Rs error at id = 0 is only a magnitude error.
 * FLUX_WB no longer sets the speed estimate (section 3). It still sets the
 * adaptive gain of (6) and the diagnostic of (14). */
#define POLE_PAIRS      3.0f
#define RS_OHM          0.06f
#define LS_H            40.0e-6f
#define FLUX_WB         0.01857f // was 0.02013

/* ---- Current limits ------------------------------------------------------
 * IQ_MAX_A is a BENCH value sized so the Jesverty SPS-6005 (60 V, 5 A,
 * 250 W) never enters constant current: at the 20 A handoff and w_e = 314
 * rad/s, copper is 1.5*20^2*0.06 = 36 W and EMF power is
 * 1.5*0.02013*314*20 = 190 W, i.e. about 4.5 A at 50 V.
 * THAT SIZING ONLY HOLDS AT THE HANDOFF SPEED. EMF power rises linearly
 * with speed at fixed iq, so at 20 A the supply reaches 250 W at about
 * 1130 rpm. On the bench supply keep the throttle modest above that, or add
 * an input power cap; on a battery it does not matter.
 * 20 A WILL NOT ACCELERATE A PROPELLER IN WATER. Measure the propeller
 * demand before the first loaded run and restore IQ_MAX_A first.
 * OC_TRIP_A stays armed in both modes and is the only current protection on
 * the bench, which is worth something ONLY IF the ACS feedback is correct in
 * sign and scale (confirmed by clamp meter: 8.2 A measured against 8.82 A
 * reported during ALIGN). */
#define IQ_MAX_A        190.0f      /* throttle full scale                    */
#define IQ_SLEW_A_S     200.0f
#define OC_TRIP_A       200.0f     /* instantaneous phase trip, rank-1 data  */
#define OC_CONFIRM_N    2u         /* consecutive samples, see section 4b    */
#define IDC_TRIP_A      70.0f     /* DC link trip, debounced                */
#define I_ALIGN_A       8.0f
#define DT_LIN_A        40.0f      /* MODULATOR dead-time blend width        */
#define DT_OBS_A        3.0f       /* OBSERVER dead-time model, narrow       */
#define DT_COMP_EN       1
#define VMAX_START_V    15.0f      /* voltage circle cap in ALIGN/OPEN_LOOP  */
#define ID_SLEW_A_S    400.0f

/* Reference anti-windup margin, section 4b. How far iq_ref is allowed to
 * lead the measured iq once the q axis is on the voltage circle. Big enough
 * that the PI still has an error to work with and the loop does not go
 * open, small enough that the reference cannot run away from a current the
 * voltage can never produce. Raising it back toward IQ_MAX_A restores the
 * old behaviour, which is the A/B test if this is ever questioned. */
#define IQ_SAT_MARGIN_A   2.0f

/* ---- Semiconductor forward drop, OBSERVER MODEL ONLY ---------------------
 * One device in each leg conducts essentially the whole period and its drop
 * opposes the current, so it enters the terminal voltage in exactly the same
 * place as the dead time:
 *     v_actual = v_cmd - (Vdt + V_DEV_A) sgn(i)
 * At the handoff the back-EMF is lambda_F*W_HANDOFF = 6.32 V, so 1.5 V of
 * unmodelled drop rotates the estimate by atan(1.5/6.32) = 0.23 rad.
 * 1.5 V is Vce(sat) plus diode Vf read off the static curves and IS A
 * PLACEHOLDER UNTIL MEASURED. The check is dbg_lambda_est, which is now a
 * real measurement because the speed no longer depends on FLUX_WB: reading
 * above 0.0201 Wb means V_DEV_A is too large. Set to 0.0f for an A/B test.
 * Applied to the observer only: compensating the device drop in the
 * modulator as well is a separate change with its own stability
 * consequences. */
#define V_DEV_A         1.5f

/* ---- DC link capacitor thermal ceiling -----------------------------------
 * One 944U221K801ACM, no temperature sensor. The two-cap analysis put
 * 92.6 A RMS in the bank at a 60.9 degC rise; rise scales as I^2, so 40 degC
 * allows 92.6*sqrt(40/60.9) = 75.1 A, and with the conservative Kolar factor
 * 0.650 [7] that is I_ph,rms <= 115.5 A, i.e. 163 A peak continuous.
 * Averaging is deliberately asymmetric: the can's own thermal constant is
 * minutes, so heating uses 60 s and cooling 180 s, and a symmetric filter
 * would let repeated bursts sit permanently under the limit while the
 * capacitor climbed. INERT at IQ_MAX_A = 20 A (reachable phase RMS 14 A);
 * it exists so that raising IQ_MAX_A later cannot cook the cap.
 * THIS IS A BOUND, NOT A PROTECTION: open loop, blind to ESR ageing and
 * ambient. Validate with a thermocouple on the can over a full-duration
 * run. */
#define IPH_RMS_CONT_A  115.5f
#define TAU_CAP_RISE_S   60.0f
#define TAU_CAP_FALL_S  180.0f
#define IQ_MAX_FLOOR_A   40.0f

/* ---- Startup profile -----------------------------------------------------
 * W_HANDOFF = 2*pi*50 rad/s electrical = 50 Hz electrical = 1000 rpm
 * mechanical at p = 3, reached in W_HANDOFF/OL_ACCEL = 2.62 s.
 * Back-EMF at that speed is lambda_F*W_HANDOFF = 6.32 V, which is what the
 * sgn(i) voltage errors of section 4 must be small against.
 * I-f has no damping of its own: the rotor hangs off the forced current
 * vector on an undamped spring at sqrt(p*kt*i/J), about 5.6 Hz on a bare
 * shaft. OL_DAMP_K injects damping by advancing the commanded angle when the
 * rotor runs ahead of it, which shrinks the load angle and the torque.
 * w_ol itself is NOT modified, so the speed gate and the RPM telemetry stay
 * clean. Useful range 0.2 to 0.5; above 0.6 the damping loop competes with
 * the tracking observer. */
#define I_STARTUP_A0    16.0f
#define I_STARTUP_A1    20.0f
#define OL_SOFT_S        0.100f
#define ALIGN_TIME_S     0.7f
#define OL_ACCEL       600.0f      /* rad/s^2 electrical                     */
#define OL_DAMP_K        0.30f
#define OL_DAMP_LIM      0.30f
#define W_HANDOFF       (2.0f * (float)M_PI * 50.0f)
#define HANDOFF_ERR_RAD  0.8f      /* 0.185 rad load angle plus margin       */
#define HANDOFF_W_FRAC   0.30f
#define SYNC_LOST_N     2000u      /* 100 ms                                 */
#define HANDOFF_HOLD_N   100u      /* 5 ms of agreement                      */
#define HANDOFF_DECAY      2u      /* decay per bad tick, not a reset        */
#define HO_WAIT_N      30000u      /* 1.5 s of trying at a HELD W_HANDOFF    */
#define BLEND_TAU_S      0.020f
#define W_SYNC_MIN      (2.0f * (float)M_PI * 10.0f)  /* 10 Hz el, 200 rpm   */

/* Post-handoff torque hold. Without it the stator current steps from the I-f
 * value to (thr/100)*iq_lim in one tick, an eight-fold torque cut at 10
 * percent throttle at the exact moment the observer must hold lock through a
 * decaying theta_off. IQ_HANDOFF_HOLD_S is 15 blend time constants, so the
 * offset is long settled before the floor lets go. Killed below
 * THR_STOP_PCT so it can never fight the dead-man. */
#define IQ_HANDOFF_FRAC   0.60f
#define IQ_HANDOFF_HOLD_S 0.30f

/* ---- Idle re-arm hold, WALL-CLOCK MILLISECONDS ---------------------------
 * A failure leaves the rotor turning at W_HANDOFF while ALIGN forces DC
 * current at theta = 0, so re-entering ALIGN quickly is the worst available
 * action. Compared against HAL_GetTick() with a signed difference, so the
 * 49-day wrap is handled. */
#define RESTART_HOLD_MS   50u      /* rider released the throttle            */
#define FAIL_HOLD_MS    1000u      /* handoff give-up, sync loss, fault clear*/

/* Phase OC uses rank-1 samples taken at the quiet centre of the ON block and
 * it is the primary protection, but it is now confirmed over OC_CONFIRM_N
 * consecutive samples (section 4b): one sample cannot be a real current
 * excursion of that size. i_dc is rank 2, whose window straddles a switching
 * edge at low duty, so one sample was never a safe basis for a latch. */
#define IDC_TRIP_N      3u

/* ---- Phase current offsets, FIXED constants ------------------------------
 * Measured ACS zero on this build, both phases. 1990 is 58 counts (7.1 A)
 * below the 2048 mid-supply nominal, inside the ACS72981 quiescent output
 * tolerance, but also what a sensor supply and an ADC reference on different
 * 3.3 V nodes would produce; if those are separate rails the offset is
 * ratiometric and a constant cannot track it.
 * ONE COUNT IS 0.122 A, and ten counts of error is 1.2 A of feedback error,
 * which eats most of the handoff angle budget. Re-measure after any change
 * to the 3V3 rail, the sensors or the ADC reference: dbg_ia_off/dbg_ib_off
 * report what the boot average actually saw, dbg_ia_drift_a/dbg_ib_drift_a
 * report the difference in amps. */
#define ACS_OFF_FIXED   2055.0f

/* ---- Boot offset calibration ---------------------------------------------
 * Measures and commits the HASS DC link zero; CHECKS the ACS averages but
 * never writes them. Both sensors idle at a known level with the gates off,
 * so an average outside these windows means the part is unpowered or
 * disconnected. What this CANNOT see: that the sensors are in the current
 * path, that IA_SIGN/IB_SIGN match the physical orientation, or that sensor
 * phase A is the phase driven by CH1. dbg_vd is the check for those. */
#define CAL_N           4000u      /* 200 ms of averaging                    */
#define CAL_SETTLE_N     400u      /* 20 ms discarded first                  */
#define ACS_OFF_MIN     1690.0f
#define ACS_OFF_MAX     2290.0f
#define HASS_OFF_MIN    2902.0f
#define HASS_OFF_MAX    3302.0f
#define CAL_RETRY_MS    1000u

/* ---- Current PI, 1 kHz bandwidth -----------------------------------------
 * Pole-zero cancellation on the stator pole: Kp = Ls*wbw = 0.2513 V/A,
 * Ki = Rs*wbw = 377 V/(A.s), giving a first-order closed loop at wbw.
 * With a persistent 8 A error the integrator gains KI_I*e*TS = 0.151 V per
 * tick and reaches a 21.5 V circle in 7 ms, so a feedback fault saturates
 * the modulator faster than any bench supply's current limit responds. */
#define WBW_RAD         (2.0f * (float)M_PI * 1000.0f)
#define KP_I            (LS_H * WBW_RAD)
#define KI_I            (RS_OHM * WBW_RAD)

/* Voltage circle radius as a fraction of Vbus, bounded by the duty clamp:
 *   0.866*VMAX_FRAC + DEADTIME_S/TS <= DUTY_MAX - 0.5  ->  VMAX_FRAC <= 0.439
 * The SVPWM linear ceiling is Vdc/sqrt(3) = 29.1 V (0.577) [1, eq. 8-15];
 * the gap is what the bootstrap and the dead time cost. 0.43*50.4 = 21.70 V
 * covers the 21.08 V that 2550 rpm at 80 A needs, and sits only 7 percent
 * above the 20.2 V that 3000 rpm at 20 A needs, so the top of the bench
 * speed range is voltage limited and sensitive to bus sag. That ceiling is
 * exactly why the reference anti-windup of section 4b exists. */
#define VMAX_FRAC       0.437f //was 0.437f

/* ---- Sliding mode observer, section 2 ------------------------------------
 * K = SMO_K0 + SMO_K_MARGIN*lambda_F*w_hat, clamped at SMO_K_MAX, which
 * satisfies the existence condition (5) with 50 percent margin at every
 * speed. The reachable EMF ceiling is set by the voltage envelope
 * (VMAX_FRAC*Vdc = 21.7 V), not by the no-load KV figure, so SMO_K_MAX = 30
 * gives 1.4x there.M
 * SMO_WC0 sets the filter (8) at standstill; the cutoff then tracks speed
 * per (10) so r = w/wc stays below 1 and the correction atan(r) stays well
 * away from pi/2. Discrete pole requires TS*wc < 1, i.e. wc < 20 krad/s,
 * satisfied by construction since w_hat is clamped at W_EST_MAX.
 * EMF_MIN_V is the modulus below which the argument of (12a) is noise; at
 * the DC gain of (8) it corresponds to roughly 15 rad/s electrical. */
#define SMO_K0          3.0f       /* V, covers voltage model error at w = 0 */
#define SMO_K_MARGIN    1.5f       /* gamma in (6)                           */
#define SMO_K_MAX      30.0f
#define SMO_WC0        (2.0f * (float)M_PI * 20.0f)   /* 125.7 rad/s         */
#define W_EST_MAX    2000.0f       /* electrical rad/s, runaway clamp        */
#define EMF_MIN_V       0.30f
#define SMO_DELAY_S    (1.5f * TS) /* sampling instant to applied voltage    */

/* ---- Angle tracking observer, section 3 ----------------------------------
 * Type-2, critically damped, gains per Xu et al. eq. 28-30: kp = 2k,
 * ki = k^2, closed-loop bandwidth 2.49*k = 782 rad/s (124 Hz).
 * ATO_EPS_LIM bounds the per-tick correction during acquisition, so a pi
 * radian error at release from the I-f seed cannot kick the integrator by
 * more than ATO_KI*ATO_EPS_LIM*TS = 4.9 rad/s in one tick.
 * Discrete margin: wb*TS = 0.039, far inside the Nyquist limit. */
#define ATO_K           (2.0f * (float)M_PI * 50.0f)  /* 314.2 rad/s         */
#define ATO_KP          (2.0f * ATO_K)                /* 628.3              */
#define ATO_KI          (ATO_K * ATO_K)               /* 98696              */
#define ATO_EPS_LIM     1.0f                          /* rad                */

/* ---- CORDIC --------------------------------------------------------------
 * q1.31 throughout. PRECISION = 6 selects 24 iterations, about 6 AHB cycles,
 * with a residual angle error below 1e-4 rad.
 *   FUNC 0 (cosine),  ARG1 = angle/pi, ARG2 = modulus, RES1 = m*cos,
 *                     RES2 = m*sin. Replaces sinf/cosf in Park.
 *   FUNC 2 (phase),   ARG1 = x, ARG2 = y, RES1 = atan2(y,x)/pi,
 *                     RES2 = sqrt(x^2+y^2). Equation (12a) and its modulus.
 *   FUNC 4 (arctan),  ARG1 = x (|x| <= 1 at SCALE = 0), RES1 = atan(x)/pi.
 *                     The filter phase correction atan(r) in (13).
 * The phase function saturates its modulus result if sqrt(x^2+y^2) >= 1, so
 * the EMF vector is scaled by 1/E_SCALE_V on the way in and the modulus is
 * scaled back on the way out. E_SCALE_V must exceed SMO_K_MAX.
 * Reading RDATA stalls the bus until the result is ready, so no polling of
 * RRDY is needed; the CSR is rewritten before each transaction, which is
 * legal because the previous results are always read first. */
#define Q31_SCALE       2147483648.0f
#define E_SCALE_V       40.0f
#define CORDIC_PREC     (6UL << 4)
#define CORDIC_CFG_COS  (0UL | CORDIC_PREC | (1UL << 19) | (1UL << 20))
#define CORDIC_CFG_PHS  (2UL | CORDIC_PREC | (1UL << 19) | (1UL << 20))
#define CORDIC_CFG_ATN  (4UL | CORDIC_PREC)

/* ---- ADC scaling ---------------------------------------------------------
 * ACS72981-200B3: 6.6 mV/A, one count = 0.1221 A, zero at ACS_OFF_FIXED.
 * HASS 300-S: Uout = Uref +/- 0.625 V at +/- 300 A, so 2.0833 mV/A and
 * 480 A/V, spanning 1.875 to 3.125 V, which fits the 3.3 V ADC with no
 * divider and rails at +384 A, above IDC_TRIP_A. Its zero is MEASURED. */
#define ADC_FS          4095.0f
#define VREF            3.3f
#define ACS_A_PER_V     (1.0f / 0.0066f)
#define ADC_A_PER_COUNT ((VREF / ADC_FS) * ACS_A_PER_V)
#define IA_SIGN         (+1.0f)
#define IB_SIGN         (+1.0f)
#define HASS_A_PER_V    480.0f
#define HASS_SIGN       (-1.0f)

/* ---- Bus voltage divider -------------------------------------------------
 * AS BUILT: 180 k top, 10 k bottom, gain 19.0 (the earlier 150 k gave 16.0
 * and clipped at 52.8 V, where a freshly charged 12S pack sits).
 *   50.4 V -> 2.653 V -> raw 3292;  3.3 V is reached at 62.7 V.
 * Source impedance 180k||10k = 9.474 k; 47.5 cycles at 42.5 MHz is 1.12 us
 * against a required 9.474k*5pF*ln(2^13) = 0.43 us. The 100 nF at the pin
 * gives tau = 0.95 ms and attenuates 20 kHz by 119x, so single-capacitor
 * bank ripple does not bias the fixed-phase sampling instant.
 * A bus error propagates TWICE: through duty normalisation and through the
 * dead-time voltage. Calibrate dbg_vbus_v_raw against a meter.
 * VBUS_FAST_ALPHA gives the ISR a 1 ms view of the bus. The 100 ms window
 * mean below is kept for the FAULT DECISION only: a supply entering
 * constant current collapses the bus in about a millisecond, and running
 * the modulator and the observer voltage model on a 100 ms average through
 * that is what makes a sag look like an angle error. */
#define VBUS_R_TOP      180.0f
#define VBUS_R_BOT      10.0f
#define VBUS_GAIN       ((VBUS_R_TOP + VBUS_R_BOT) / VBUS_R_BOT)
#if LAB_TEST_MODE
#define VBUS_PLAUS_MIN  10.0f
#else
#define VBUS_PLAUS_MIN  20.0f
#endif
#define VBUS_PLAUS_MAX  60.0f
#define VBUS_FALLBACK   48.0f
#define VBUS_RAW_CLIP   4090u
#define VBUS_BAD_WIN    3u
#define VBUS_GOOD_WIN   3u
#define VBUS_FAST_ALPHA  0.05f   /* tau = TS/alpha = 1 ms */

/* ---- Throttle / dead-man -------------------------------------------------*/
#define THR_START_PCT   8.0f
#define THR_STOP_PCT    5.0f
#define BLE_TIMEOUT_MS  500

/* ---- NTC -----------------------------------------------------------------
 * FM600TU-3A module thermistor, R25 = 100 k, B = 4000 K.
 * TOPOLOGY AS BUILT: 3V3 -- R_FIXED -- node(PB14) -- NTC -- GND, so the node
 * voltage FALLS as the module heats, R_NTC = R_FIXED*raw/(FS-raw), an open
 * NTC gives raw = FS and a short gives 0. Report equation 40 and figure 14
 * show the opposite topology and must be corrected.
 * With the 10 k part the mean falls about 15 counts per degree near room
 * temperature (3780 at 21 degC) and sits 300 counts below NTC_RAW_HI, which
 * is thin on a cold morning; 33 k is the geometric mean of R25 and R90 and
 * is the better part. Change the resistor and NTC_R_FIXED together. */
#define NTC_R_FIXED    10000.0f
#define NTC_R25        100000.0f
#define NTC_BETA        4000.0f
#define NTC_T25_K        298.15f
#define NTC_RAW_LO        15u
#define NTC_RAW_HI      4080u
#define TEMP_TRIP_C       90.0f
#define NTC_IIR_ALPHA      0.3f

/* Fault decision on WINDOWS, not single samples: 2000 conversions per
 * window, a window is bad when more than half were out of range, and bad
 * samples are excluded from the mean. Entry and exit use separate counters
 * that reset on the opposite result, so the flag cannot change twice inside
 * 1.5 s and ERR cannot flash on the decision boundary. */
#define NTC_BAD_FRAC_NUM  1u
#define NTC_BAD_FRAC_DEN  2u
#define NTC_BAD_WIN      10u
#define NTC_GOOD_WIN     10u
#define OT_CONFIRM_N      3u

#define BLE_RX_BUF_LEN   64

/* ---- Telemetry -----------------------------------------------------------
 * Two frame types share one buffer, one busy guard and one queue path.
 * TELEM_TX_LEN must hold the LONGER frame: status worst case 38, debug 32.
 * Budget: 38 B at 10 Hz plus 32 B at 5 Hz is 540 B/s of the 960 B/s the
 * HM-10 gives at 9600 baud, against 140 B/s of inbound throttle. */
#define TELEM_MS_RUN     100u
#define TELEM_MS_IDLE    250u
#define TELEM_T10_FAULT  9999
#define TELEM_IDC_ABSENT (-9999)
#define TELEM_TX_LEN     48
#define TELEM_STUCK_MS   500u
#if LAB_TEST_MODE
#define TELEM_LEAD_CH   'L'
#else
#define TELEM_LEAD_CH   'S'
#endif
#define TELEM_DBG_LEAD_CH  'D'
#define TELEM_DBG_MS_MULT  2u

/* ---- Fault codes (must match FAULT_TXT on the Pico) ----------------------*/
#define FAULT_NONE      0u
#define FAULT_PHASE_OC  1u
#define FAULT_OVERTEMP  2u
#define FAULT_ACS_SAT   3u
#define FAULT_DCLINK_OC 4u

/* ---- Green LED -----------------------------------------------------------*/
#define LED_EVAL_MS         20u
#define LED_SOLID_PCT       95.0f
#define LED_HALF_TICKS_MAX  25.0f
#define LED_HALF_TICKS_MIN   2.0f
#define LED_HB_ON_TICKS      2u

/* ---- Trace recorder, section 5 -------------------------------------------
 * TRACE_EN 0 removes every byte of it: the buffer, the ISR call and the
 * UART. Leave it 1 on the bench and set it 0 for the water build.
 *
 * RAM: TRACE_DEPTH * TRACE_CH_N * 2 = 672 * 12 * 2 = 16128 bytes of the
 * 32 kB on a G431RB, i.e. HALF THE SRAM ON A DEBUG FEATURE. TRACE_DEPTH was
 * cut from 1024 to 896 when the iq_ref channel was added, so the buffer is
 * the same size it has always been; putting DEPTH back to 1024 costs
 * 2304 bytes more. CHECK THE LINKER "RAM used" LINE after the first build
 * and drop to 768 (13824 B) if it is tight. Everything else static in this
 * file is under 1.5 kB.
 *
 * RECORD LENGTH (TRACE_DEPTH 672, Ts = 50 us)
 *   TRACE_DECIM   1 -> 20 kHz,  33.6 ms   current loop step, chatter
 *   TRACE_DECIM  20 ->  1 kHz,  0.67 s    handoff detail, sync loss
 *   TRACE_DECIM 200 -> 100 Hz,  6.72 s    whole run, speed and power
 * With TRACE_PRE = 80 the post-trigger window is 5.92 s at decim 200.
 *
 * TRACE_PRE is how many samples are kept from BEFORE the trigger instant.
 * It is 80 and not 200 because the trigger fires at the ALIGN to OPEN_LOOP
 * transition and ALIGN lasts ALIGN_TIME_S = 0.7 s, which is only 70 rows at
 * decim 200. Anything above that can never be filled, and since the
 * post-trigger window is TRACE_DEPTH - TRACE_PRE, every unusable pre-trigger
 * slot costs a usable one at the end of the run. At 80 the record runs
 * 8.16 s past the trigger, which covers the 2.62 s I-f ramp plus the full
 * 1.5 s HO_WAIT_N acquisition attempt with margin.
 *
 * TRACE_BAUD 921600 is what the ST-LINK VCP will carry; drop to 115200 if
 * the dump arrives corrupted. A full 896-row record is about 50 kB, i.e.
 * 0.55 s of wall clock at 921600, spent in IDLE or FAULT with the gates off.
 * TRACE_LINES_PER_PASS bounds how long ONE main-loop pass blocks inside
 * HAL_UART_Transmit: 8 lines of about 55 bytes is 440 bytes, 4.8 ms at
 * 921600. It does not change the total dump time, which is set by the baud
 * rate; it only bounds how long BLE packets sit unparsed in the DMA buffer
 * between passes. */
#define TRACE_EN             1
#define TRACE_CH_N           12u
#define TRACE_DEPTH          672u      /* 672 x 12 x 2 = 16128 B, unchanged */
#define TRACE_DECIM          200u
#define TRACE_PRE            80u
#define TRACE_LINES_PER_PASS 8u
#define TRACE_BAUD           921600u

/* Electrical rad/s -> MECHANICAL rpm, for the two speed channels. The
 * recorder logs mechanical rpm because every other number on this bench
 * (the telemetry rpm field, the 3000 rpm operating point, the propeller) is
 * mechanical, and because it makes the trace directly comparable with the
 * status frame. = 3.1831 at p = 3. */
#define RPM_PER_WE      (60.0f / (2.0f * (float)M_PI * POLE_PAIRS))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define WRAP_2PI(x)  do { if ((x) >  (float)M_PI) (x) -= 2.0f*(float)M_PI; \
                          if ((x) < -(float)M_PI) (x) += 2.0f*(float)M_PI; } while (0)
#define CLAMPF(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define TICK_REACHED(now, deadline)  ((int32_t)((now) - (deadline)) >= 0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* ---- Published state ---- */
volatile float   module_temp_c    = 0.0f;
volatile uint8_t ntc_fault        = 0;
volatile uint8_t vbus_sense_fault = 1;      /* assume bad until proven good */
volatile uint8_t idc_sense_ok     = 0;
volatile float   vbus_v           = VBUS_FALLBACK;
static float vbus_fast = VBUS_FALLBACK;
volatile float   idc_a            = 0.0f;

/* Effective iq ceiling, written by the 100 ms loop and read by the ISR.
   Single aligned 32-bit accesses on Cortex-M4, so no critical section. The
   derate can only pull it DOWN. */
volatile float iq_max_eff = IQ_MAX_A;

/* Window accumulators, filled at 20 kHz and drained by the 100 ms loop.
   ntc_acc_sum holds only IN-RANGE samples so the mean is not dragged by
   switching spikes. Worst case 2000*4095 = 8.19e6, well inside uint32. */
static volatile uint32_t ntc_acc_sum  = 0;
static volatile uint32_t ntc_acc_good = 0;
static volatile uint32_t ntc_acc_bad  = 0;
static volatile uint32_t vb_acc_sum   = 0;
static volatile uint32_t vb_acc_n     = 0;
static volatile uint32_t vb_acc_clip  = 0;

/* Phase current sum of squares. One instantaneous sample of a sinusoid taken
   ten times a second is a random point on the waveform, so the display would
   swing between zero and the peak on a steady machine. Feeds both the
   telemetry frame and the capacitor thermal estimate. Worst case sum is
   2000*190^2 = 7.2e7 against about seven significant digits in a float, so
   do not lengthen the window without moving to double. */
static volatile float    irms_acc_a = 0.0f;
static volatile float    irms_acc_b = 0.0f;
static volatile float    irms_acc_c = 0.0f;
static volatile uint32_t irms_acc_n = 0;

/* Electrical speed accumulator for the RPM field. Same window and the same
   critical section as the RMS currents, for the same reason: the tracking
   observer has a 124 Hz bandwidth and sampling it once per 100 ms returns a
   random point on whatever the estimate is doing between 10 and 124 Hz.
   Worst case sum is 2000*2000 = 4e6, well inside a float. */
static volatile float    w_acc   = 0.0f;
static volatile uint32_t w_acc_n = 0;

volatile float ia_rms = 0.0f, ib_rms = 0.0f, ic_rms = 0.0f;
volatile float motor_rpm = 0.0f;

/* MUST be static: with DMA the UART reads it long after the call returns. */
static char telem_tx[TELEM_TX_LEN];

/* ---- BLE ---- */
uint8_t  ble_rx_buf[BLE_RX_BUF_LEN];               /* DMA writes here       */
static uint8_t ble_rx_copy[BLE_RX_BUF_LEN];        /* callback copies here  */
volatile uint16_t ble_rx_len = 0;
volatile uint8_t  ble_rx_ready = 0;
volatile float    throttle_percent = 0.0f;
volatile uint32_t last_valid_rx_tick = 0;
static char    line_buf[32];
static uint8_t line_idx = 0;

/* ---- FOC state ---- */
volatile foc_state_t foc_state  = FOC_OFFSET_CAL;
volatile uint8_t     fault_code = FAULT_NONE;

/* Earliest HAL tick at which IDLE may hand the drive back to ALIGN. */
static volatile uint32_t restart_block_until = 0;

/* Phase offsets are compile-time constants; only the HASS zero is measured,
   because it is not known ahead of time and it gates the start interlock. */
static float    ia_off = ACS_OFF_FIXED, ib_off = ACS_OFF_FIXED;
static float    idc_off = 3102.0f;
static float    off_acc_a = 0.0f, off_acc_b = 0.0f, off_acc_dc = 0.0f;
static uint32_t off_n = 0;
static uint32_t cal_settle = 0;

volatile float ia = 0, ib = 0, ic = 0;
volatile float id_meas = 0, iq_meas = 0;

static float id_ref = 0, iq_ref = 0;
static float id_int = 0, iq_int = 0;
static float v_alpha = 0, v_beta = 0;   /* observer input, see section 4     */

static float theta = 0;                 /* angle used by Park this cycle     */
static float theta_off = 0;             /* handoff blend offset              */
static float align_t = 0;
static float ol_t = 0;
static float theta_ol = 0;
volatile float w_ol = 0;                /* read by the slow loop for RPM     */

/* Sliding mode observer states */
static float ia_hat = 0, ib_hat = 0;    /* i_hat in (3)                      */
static float e_alpha = 0, e_beta = 0;   /* e_hat, the filtered z of (8)      */

/* Angle tracking observer states, section 3. theta_trk tracks the CORDIC
   argument and coasts on w_est if the EMF becomes unusable. w_est is the
   ONLY speed estimate in the drive and is independent of FLUX_WB. */
static float theta_trk = 0;
volatile float w_est = 0;               /* electrical rad/s                  */

/* Handoff, sync and trip counters. ALL file scope and ALL cleared in
   foc_reset(): as function statics they survived across runs, so a run that
   ended with a partial sync_lost_cnt started the next run most of the way to
   a shutdown. oc_cnt is the phase OC confirmation counter of section 4b and
   belongs to the same family. */
static uint16_t handoff_cnt   = 0;
static uint16_t ho_wait       = 0;
static uint16_t sync_lost_cnt = 0;
static uint16_t idc_trip_cnt  = 0;
static uint16_t oc_cnt        = 0;
static uint16_t emf_bad_cnt   = 0;
static float    iq_floor      = 0.0f;

/* Filtered measured iq for the reference anti-windup of section 4b. A 1 ms
   IIR, so the cap follows the real current without carrying the per-tick
   observer ripple into the reference. */
static float    iq_meas_f     = 0.0f;

/* ---- Trace recorder, section 5 ------------------------------------------
 * huart2 is declared HERE and not by CubeMX, and is brought up entirely by
 * trace_uart_init(), so the .ioc is untouched and a regeneration cannot
 * remove it. USART2 is TX only in practice; RX is configured but unused.
 *--------------------------------------------------------------------------*/
#if TRACE_EN
UART_HandleTypeDef huart2;
static uint8_t trace_uart_ok = 0;

/* CHANNEL TABLE. The one thing to edit when changing what is logged. Keep
   the order identical to the array built at the end of foc_isr(), keep
   TRACE_CH_N in step, and keep avg = 0 on any wrapped angle or any channel
   that is not a continuous quantity.
   Scales are INTEGER (see trace_ch_t):
     theta x10000 = 0.1 mrad/count (+/-pi is +/-31416, fits int16)
     rpm   x4     = 0.25 rpm/count, clips at 8191 rpm MECHANICAL
     i     x100   = 10 mA/count, clips at +/-327 A
     state x1     = the foc_state_t enum value
   A channel that clips saturates rather than wrapping, so it shows as a flat
   top and never as a sign reversal.

   WHY iq_ref. It is the torque command the throttle actually produced after
   the slew limiter, the iq_floor hold, iq_max_eff and the section 4b
   anti-windup have all had their say, so the gap between iq_ref and iq is
   the only direct view of whether the current loop is keeping up. On the
   voltage circle that gap should stay near IQ_SAT_MARGIN_A; if it opens out
   toward IQ_MAX_A the anti-windup is not engaging and the drive is back in
   the regime that trips.

   WHY i_s AND NOT ia. A box average of ia over a 10 ms decimation window at
   50 Hz electrical is close to zero, and sub-sampling it is a random point
   on the waveform. |i_alpha + j i_beta| is the PEAK PHASE CURRENT directly
   under the amplitude-invariant Clarke of section 1, it box-averages
   meaningfully at any decimation, and unlike id/iq it does not depend on
   theta: if the observer angle is wrong, id and iq swirl into each other
   while i_s still reports what is really in the machine. Phase RMS is
   i_s/sqrt(2). For a true waveform capture set TRACE_DECIM = 1, change this
   entry to { 100, 0u, "ia" } and log la instead.

   WHY state. It puts the ALIGN -> OPEN_LOOP -> CLOSED_LOOP instants on the
   plot, so theta_est against theta_ol becomes readable and a handoff
   give-up (HO_WAIT_N expiring back to IDLE) is distinguishable from a
   successful run at a glance. STATE 5 (FAULT) CAN NEVER APPEAR: foc_trip()
   calls trace_stop() and then every trip path does goto isr_exit, which
   skips the recorder, so a fault record simply ends and the final row holds
   the state that was live when it tripped. */
static const trace_ch_t trace_ch[TRACE_CH_N] = {
    {     4, 1u, "rpm_est"   },  /* tracking observer, MECHANICAL rpm      */
    {     4, 1u, "rpm_ol"    },  /* commanded I-f ramp, MECHANICAL rpm     */
    {   100, 1u, "id"        },
    {   100, 1u, "iq"        },
    {   100, 1u, "iq_ref"    },  /* throttle torque command, post-limits   */
    {   100, 1u, "i_s"       },  /* |i_alpha + j i_beta|, peak phase amps  */
    {  1000, 1u, "vd"        },  /* PI output, 1 mV/count, clips +/-32.7 V */
    {  1000, 1u, "vq"        },
    {   100, 1u, "idc"       },  /* HASS DC link, 10 mA/count              */
    {   500, 1u, "vbus"      },  /* vbus_fast, 2 mV/count, clips 65.5 V    */
    {   100, 0u, "temp"      },  /* module NTC degC, already 100 ms filtered*/
    {     1, 0u, "state"     },  /* foc_state_t, 0..4                      */
};

static int16_t tr_buf[TRACE_DEPTH][TRACE_CH_N];
static float   tr_acc[TRACE_CH_N];

static volatile trace_state_t tr_st = TRACE_OFF;
static volatile uint16_t tr_wr        = 0;
static volatile uint16_t tr_filled    = 0;
static volatile uint16_t tr_post      = 0;
static volatile uint16_t tr_dcnt      = 0;
static volatile uint16_t tr_decim     = 1;
static volatile uint16_t tr_pre       = 0;
static volatile uint16_t tr_trig_wr   = 0xFFFFu;
static float             tr_inv_decim = 1.0f;

/* Computed once at freeze, read by the dump */
static uint16_t tr_start    = 0;
static int32_t  tr_trig_row = -1;

/* Dump cursor */
static uint16_t tr_dump_i   = 0;
static uint8_t  tr_hdr_done = 0;
static char     tr_line[192];
#endif /* TRACE_EN */

/* ---- Debug, in the order to read them during bring-up --------------------
 *  dbg_vd/vq/vmax      the voltage the current PI is asking for against the
 *                      radius it is allowed. READ FIRST if the supply enters
 *                      constant current. Frozen at the tripping tick because
 *                      they are written after every trip test. vq is also
 *                      the independent cross-check on the reported speed:
 *                      vq = lambda_F*w_e + Rs*iq, so 20 V at 20 A means
 *                      3000 rpm and 15.8 V means 2300 rpm.
 *  dbg_v_sat           bit0 = d axis on its limit, bit1 = q axis on the
 *                      voltage circle. NOT PURELY DIAGNOSTIC ANY MORE: the
 *                      reference anti-windup of section 4b reads bit 1.
 *  dbg_iq_ref          the torque command after every limiter, i.e. what
 *                      the throttle actually achieved. Compare with
 *                      iq_meas: a gap near IQ_SAT_MARGIN_A at high speed is
 *                      the anti-windup holding; a gap of many amps means it
 *                      is not engaging.
 *  dbg_ia_off/ib_off   what the BOOT average measured, in ADC counts; the
 *                      drive itself runs on ACS_OFF_FIXED.
 *  dbg_ia_drift_a      that measurement minus ACS_OFF_FIXED, in AMPS, i.e.
 *  dbg_ib_drift_a      the feedback error the constant carries. Tenths of an
 *                      amp is ordinary thermal drift; whole amps means the
 *                      constant is wrong for this build or temperature.
 *  dbg_ho_derr         observer-vs-forced angle error at handoff speed. Log
 *                      it through a whole ramp before touching a gain.
 *  dbg_ho_best         smallest |derr| since reset: says whether the observer
 *                      converged and was gated by HANDOFF_HOLD_N, or never
 *                      converged at all.
 *  dbg_ksw             the adaptive switching gain of (6), in volts. Compare
 *                      against lambda_F*w_est: the ratio is the existence
 *                      margin of (5) and must stay above 1.
 *  dbg_smo_chat        |s| averaged over the last tick, in amps, i.e. the
 *                      quasi-sliding band of (11). Expect about Ts*K/Ls,
 *                      which is large by design; see section 2.
 *  dbg_ato_eps         tracking observer error in radians. Should sit near
 *                      zero in steady state. A non-zero MEAN means the
 *                      tracker is not keeping up and the RPM cannot be
 *                      trusted; ripple is normal and is what the loop is
 *                      there to filter.
 *  dbg_lambda_est      measured flux linkage from (14). This is now a REAL
 *                      measurement, not a tautology: it should settle near
 *                      FLUX_WB. Reading high means the voltage model is
 *                      over-driving the observer, i.e. V_DEV_A is too large.
 *  dbg_oc_singles      how many times a SINGLE sample passed OC_TRIP_A
 *                      without a second one confirming it. Non-zero means
 *                      the rank-1 sample is picking up switching noise and
 *                      the old undebounced trip was firing on it.
 *  dbg_emf_bad_max     longest run of unusable-EMF ticks. Approaching
 *                      SYNC_LOST_N means the observer is dropping out rather
 *                      than the machine stalling.
 *  dbg_isr_cycles      budget 8500 at 170 MHz / 20 kHz. The trace recorder
 *                      adds of order 180 at nine channels; compare max
 *                      before and after rather than assuming.
 *  dbg_trace_state     0 = off/dumped, 1 = recording, 2 = frozen and waiting
 *                      for the main loop to dump it.
 */
volatile uint8_t  dbg_lab_mode = (uint8_t)LAB_TEST_MODE;
volatile float    dbg_duty_a = 0.5f;
volatile uint32_t dbg_isr_cycles = 0;
volatile uint32_t dbg_isr_cycles_max = 0;

volatile float    dbg_vd = 0.0f;
volatile float    dbg_vq = 0.0f;
volatile float    dbg_vmax = 0.0f;
volatile uint8_t  dbg_v_sat = 0;        /* bit0 = d on limit, bit1 = q      */

volatile float    dbg_ho_derr = 0.0f;
volatile float    dbg_ho_best = 10.0f;
volatile float    dbg_ksw = 0.0f;
volatile float    dbg_smo_chat = 0.0f;
volatile float    dbg_ato_eps = 0.0f;
volatile float    dbg_lambda_est = 0.0f;

volatile float    dbg_iq_ref = 0.0f;
volatile uint32_t dbg_oc_singles = 0;

volatile uint32_t dbg_telem_sent = 0;
volatile uint32_t dbg_telem_skipped = 0;
volatile uint32_t dbg_rx_rearm = 0;
volatile uint32_t dbg_cal_retry = 0;

volatile float    dbg_ia_off = ACS_OFF_FIXED;
volatile float    dbg_ib_off = ACS_OFF_FIXED;
volatile float    dbg_idc_off = 3102.0f;
volatile float    dbg_ia_drift_a = 0.0f;
volatile float    dbg_ib_drift_a = 0.0f;
volatile uint32_t dbg_cal_done = 0;

volatile float    dbg_iph_slow = 0.0f;
volatile float    dbg_iq_max_eff = IQ_MAX_A;

volatile uint16_t dbg_ntc_raw_mean = 0;
volatile uint8_t  dbg_ntc_bad_pct = 0;
volatile uint16_t dbg_vbus_raw_mean = 0;
volatile uint8_t  dbg_vbus_clip_pct = 0;
volatile float    dbg_vbus_v_raw = 0.0f;

volatile uint16_t dbg_emf_bad_max = 0;
volatile uint16_t dbg_sync_exits = 0;
volatile uint16_t dbg_ho_fails = 0;
volatile float    dbg_iq_floor = 0.0f;

volatile uint8_t  dbg_trace_state = 0;
volatile uint16_t dbg_trace_n = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
static void  foc_hw_guard(void);
static void  foc_adc_injected_config(void);
static void  gates_enable(void);
static void  gates_disable(void);
static void  foc_reset(void);
static void  foc_trip(uint8_t code);
static void  foc_cal_arm(void);
static void  foc_to_idle(uint32_t hold_ms);
static void  foc_isr(void);
static void  cap_limit_update(void);
static float ntc_temp_c(uint16_t raw);
static void  parse_ble_bytes(const uint8_t *buf, uint16_t len);
static void  ble_rx_arm(void);
static void  ble_rx_watchdog(void);
static void  telem_send(void);
static void  telem_send_dbg(void);
static void  led_update(uint32_t now);
#if TRACE_EN
static void  trace_uart_init(void);
static void  trace_arm(uint16_t decimation, uint16_t pre_samples);
static void  trace_put(const float *v);
static void  trace_trigger(void);
static void  trace_stop(void);
static void  trace_service(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ================================================== CORDIC primitives =====
 * See the CORDIC block in the defines. All three helpers are blocking on the
 * RDATA read, which stalls the AHB for the 6-cycle pipeline rather than
 * polling RRDY.
 *--------------------------------------------------------------------------*/
static inline int32_t q31_from_f(float x)
{
    if (x >=  1.0f) return  0x7FFFFFFF;
    if (x <= -1.0f) return -0x7FFFFFFF;
    return (int32_t)(x * Q31_SCALE);
}

/* cos and sin of theta in one transaction, theta already wrapped to +/-pi */
static inline void cordic_sincos(float th, float *ps, float *pc)
{
    CORDIC->CSR   = CORDIC_CFG_COS;
    CORDIC->WDATA = (uint32_t)q31_from_f(th * (1.0f / (float)M_PI));
    CORDIC->WDATA = 0x7FFFFFFFu;                    /* unit modulus         */
    int32_t rc = (int32_t)CORDIC->RDATA;
    int32_t rs = (int32_t)CORDIC->RDATA;
    *pc = (float)rc * (1.0f / Q31_SCALE);
    *ps = (float)rs * (1.0f / Q31_SCALE);
}

/* Equation (12a): returns atan2(y, x) in radians and, through pmag, the
   modulus sqrt(x^2 + y^2) rescaled back to volts. */
static inline float cordic_phase(float y, float x, float *pmag)
{
    CORDIC->CSR   = CORDIC_CFG_PHS;
    CORDIC->WDATA = (uint32_t)q31_from_f(x * (1.0f / E_SCALE_V));
    CORDIC->WDATA = (uint32_t)q31_from_f(y * (1.0f / E_SCALE_V));
    int32_t rth = (int32_t)CORDIC->RDATA;
    int32_t rmg = (int32_t)CORDIC->RDATA;
    *pmag = (float)rmg * (E_SCALE_V / Q31_SCALE);
    return (float)rth * ((float)M_PI / Q31_SCALE);
}

/* atan(x) for |x| <= 1, which is guaranteed for r = w/(wc0 + w). */
static inline float cordic_atan(float x)
{
    CORDIC->CSR   = CORDIC_CFG_ATN;
    CORDIC->WDATA = (uint32_t)q31_from_f(x);
    return (float)(int32_t)CORDIC->RDATA * ((float)M_PI / Q31_SCALE);
}

#if TRACE_EN
/* ================================================== TRACE RECORDER =======
 * Section 5 of the header block. Nothing here touches the .ioc: the USART2
 * clock, the PA2/PA3 alternate function and the handle are all set up below.
 *--------------------------------------------------------------------------*/

/* ------------------------------------------------------- USART2 bring-up --
 * PA2 = USART2_TX, PA3 = USART2_RX, AF7. On the NUCLEO-G431RB these two pins
 * are wired to the ST-LINK virtual COM port, so the dump appears as a COM
 * port on the PC with no extra wiring. They are free: nothing else in this
 * firmware claims PA2 or PA3.
 * CubeMX's board default would have used LPUART1 on the same two pins; that
 * makes no difference to the ST-LINK, which only sees the pins. If the .ioc
 * is ever changed to enable LPUART1, delete this function and use that
 * handle instead, or the two peripherals will fight over the pins.
 * A FAILURE HERE DISABLES THE RECORDER AND NOTHING ELSE. Error_Handler()
 * would park the drive in a while(1) for the sake of a debug port.
 *--------------------------------------------------------------------------*/
static void trace_uart_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = TRACE_BAUD;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    /* HAL_UART_MspInit in stm32g4xx_hal_msp.c only knows about USART1, so it
       falls through for this instance; everything it would have done is
       above. */
    trace_uart_ok = (HAL_UART_Init(&huart2) == HAL_OK) ? 1u : 0u;
}

/* ------------------------------------------------------------ arm --------
 * Circular recording starts immediately, so the buffer always holds the last
 * TRACE_DEPTH*decim ticks whether or not a trigger ever arrives. Called from
 * foc_reset(), i.e. once per throttle-up.
 *--------------------------------------------------------------------------*/
static void trace_arm(uint16_t decimation, uint16_t pre_samples)
{
    tr_st = TRACE_OFF;                 /* stop the writer before touching it */

    tr_wr = 0u; tr_filled = 0u; tr_post = 0u; tr_dcnt = 0u;
    tr_trig_wr = 0xFFFFu; tr_trig_row = -1; tr_start = 0u;
    tr_dump_i = 0u; tr_hdr_done = 0u;

    tr_decim     = decimation ? decimation : 1u;
    tr_inv_decim = 1.0f / (float)tr_decim;
    tr_pre       = (pre_samples < TRACE_DEPTH) ? pre_samples
                                               : (uint16_t)(TRACE_DEPTH - 1u);
    for (uint8_t i = 0u; i < TRACE_CH_N; i++) tr_acc[i] = 0.0f;

    tr_st = TRACE_RUN;
    dbg_trace_state = (uint8_t)tr_st;
}

/* --------------------------------------------------------- freeze --------
 * Computes where the ring unwraps and where the trigger landed inside it.
 * Called from the ISR (post-trigger expiry) and from foc_trip(), both of
 * which run at ISR priority, so tr_wr cannot move underneath it.
 *--------------------------------------------------------------------------*/
static inline void trace_freeze(void)
{
    tr_start    = (tr_filled >= TRACE_DEPTH) ? tr_wr : 0u;
    tr_trig_row = (tr_trig_wr == 0xFFFFu) ? -1
                : (int32_t)(((uint32_t)tr_trig_wr + TRACE_DEPTH - tr_start)
                            % TRACE_DEPTH);
    tr_st = TRACE_FULL;
    dbg_trace_state = (uint8_t)tr_st;
    dbg_trace_n = tr_filled;
}

/* ------------------------------------------------------------- record ----
 * Cost: nine float multiplies, nine compares, eighteen stores, plus the
 * conditional accumulate. Roughly 120 cycles of the 8500 budget.
 *--------------------------------------------------------------------------*/
static void trace_put(const float *v)
{
    if (tr_st != TRACE_RUN) return;

    for (uint8_t i = 0u; i < TRACE_CH_N; i++) {
        if (trace_ch[i].avg) tr_acc[i] += v[i];
    }

    if (++tr_dcnt < tr_decim) return;
    tr_dcnt = 0u;

    int16_t *row = tr_buf[tr_wr];
    for (uint8_t i = 0u; i < TRACE_CH_N; i++) {
        float x = (trace_ch[i].avg ? tr_acc[i] * tr_inv_decim : v[i])
                  * (float)trace_ch[i].k;
        tr_acc[i] = 0.0f;
        row[i] = (x >=  32767.0f) ?  (int16_t) 32767
               : (x <= -32767.0f) ?  (int16_t)-32767
                                  :  (int16_t)x;
    }

    if (++tr_wr >= TRACE_DEPTH) tr_wr = 0u;
    if (tr_filled < TRACE_DEPTH) tr_filled++;

    if (tr_post != 0u && --tr_post == 0u) trace_freeze();
}

/* Marks t = 0 and lets the ring run on for (DEPTH - pre) more samples. */
static void trace_trigger(void)
{
    if (tr_st == TRACE_RUN && tr_post == 0u) {
        tr_trig_wr = tr_wr;
        tr_post    = (uint16_t)(TRACE_DEPTH - tr_pre);
    }
}

/* Freeze now, keeping the pre-trigger history. Called from foc_trip() so a
   fault capture shows the run-up to the trip instead of nothing. */
static void trace_stop(void)
{
    if (tr_st == TRACE_RUN) trace_freeze();
}

/* --------------------------------------------------------------- dump ----
 * Blocking transmit, TRACE_LINES_PER_PASS lines per main-loop pass, so the
 * loop keeps servicing the BLE receiver and the dead-man between chunks.
 * CALLED ONLY WITH THE GATES OFF; the guard is at the call site.
 *
 * FORMAT. Integers only, because newlib-nano omits floating point printf
 * unless the linker flag is set, and because int16 counts are half the
 * bytes. The scale factors ride in the header and the PC divides them out.
 *   #trace,ver=2,n=<samples>,ch=<channels>,decim=<d>,fpwm=20000,trig=<row>
 *   #scale,4,4,100,100,100,100,1000,1000,100,500,100,1
 *   n,rpm_est,rpm_ol,id,iq,iq_ref,i_s,vd,vq,idc,vbus,temp,state
 *   0,12,0,801,-3,2000,801,48,0,12,25200,2150,2
 *   ...
 *   #end
 * t = (n - trig) * decim / fpwm, so t = 0 is the trigger instant and the
 * pre-trigger samples carry negative time.
 *--------------------------------------------------------------------------*/
static void trace_tx(int n)
{
    if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t *)tr_line,
                                 (uint16_t)n, 200u);
}

static void trace_service(void)
{
    if (!trace_uart_ok || tr_st != TRACE_FULL) return;

    if (!tr_hdr_done) {
        int n = snprintf(tr_line, sizeof(tr_line),
                         "#trace,ver=2,n=%u,ch=%u,decim=%u,fpwm=%u,trig=%ld\n",
                         (unsigned)tr_filled, (unsigned)TRACE_CH_N,
                         (unsigned)tr_decim, (unsigned)F_PWM_HZ,
                         (long)tr_trig_row);
        trace_tx(n);

        n = snprintf(tr_line, sizeof(tr_line), "#scale");
        for (uint8_t i = 0u; i < TRACE_CH_N; i++)
            n += snprintf(tr_line + n, sizeof(tr_line) - n, ",%d",
                          (int)trace_ch[i].k);
        n += snprintf(tr_line + n, sizeof(tr_line) - n, "\n");
        trace_tx(n);

        n = snprintf(tr_line, sizeof(tr_line), "n");
        for (uint8_t i = 0u; i < TRACE_CH_N; i++)
            n += snprintf(tr_line + n, sizeof(tr_line) - n, ",%s",
                          trace_ch[i].name);
        n += snprintf(tr_line + n, sizeof(tr_line) - n, "\n");
        trace_tx(n);

        tr_hdr_done = 1u;
        return;                         /* header only on this pass          */
    }

    for (uint16_t k = 0u; k < TRACE_LINES_PER_PASS && tr_dump_i < tr_filled;
         k++, tr_dump_i++) {
        uint16_t row = (uint16_t)(((uint32_t)tr_start + tr_dump_i)
                                  % TRACE_DEPTH);
        int n = snprintf(tr_line, sizeof(tr_line), "%u", (unsigned)tr_dump_i);
        for (uint8_t i = 0u; i < TRACE_CH_N; i++)
            n += snprintf(tr_line + n, sizeof(tr_line) - n, ",%d",
                          (int)tr_buf[row][i]);
        n += snprintf(tr_line + n, sizeof(tr_line) - n, "\n");
        trace_tx(n);
    }

    if (tr_dump_i >= tr_filled) {
        trace_tx(snprintf(tr_line, sizeof(tr_line), "#end\n"));
        tr_st = TRACE_OFF;              /* re-armed by the next foc_reset()  */
        dbg_trace_state = (uint8_t)tr_st;
    }
}
#endif /* TRACE_EN */

/* -------------------------------------------------- regeneration guard ----
 * Re-applies every hardware setting a CubeMX regeneration could revert and
 * that would be dangerous if lost. Called once from USER CODE BEGIN 2, after
 * all MX_ init and before anything is started. The analog GPIO block is not
 * redundant against HAL_ADC_MspInit: MspInit configures those pins from the
 * .ioc pinout, so a pin no longer claimed by a configured channel can
 * quietly come back as a digital input.
 *--------------------------------------------------------------------------*/
static void foc_hw_guard(void)
{
    GPIO_InitTypeDef g = {0};

    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    g.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;      /* PA0 PA1 PA4      */
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin  = GPIO_PIN_0 | GPIO_PIN_14;                  /* PB0 PB14         */
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);   /* GATE_EN low */
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOC, &g);

    /* TIM1 critical registers, written directly so a reverted .ioc cannot
       silently remove the dead time or the centre-aligned mode. CEN is clear
       here because PWM has not been started yet. */
    TIM1->CR1  = (TIM1->CR1 & ~(TIM_CR1_CMS | TIM_CR1_DIR)) | TIM_CR1_CMS_0;
    TIM1->PSC  = 0;
    TIM1->ARR  = PWM_ARR;
    TIM1->RCR  = 1;                                     /* TRGO every 50 us */
    TIM1->CR2  = (TIM1->CR2 & ~TIM_CR2_MMS) | (0x2UL << TIM_CR2_MMS_Pos);
    TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG_Msk) | DEADTIME_DTG;
    TIM1->CCR1 = PWM_ARR / 2;
    TIM1->CCR2 = PWM_ARR / 2;
    TIM1->CCR3 = PWM_ARR / 2;
    TIM1->EGR  = TIM_EGR_UG;
}

/* ------------------------------------------- injected sequence config ----
 * Entirely in user code so it survives regeneration. Must run before
 * HAL_ADCEx_InjectedStart, i.e. while JADSTART = 0.
 *   ADC1 rank 1  IN1  (PA0)   i_a     6.5 cyc   0.00 - 0.45 us after TRGO
 *   ADC1 rank 2  IN15 (PB0)   i_dc   47.5 cyc   0.45 - 1.86 us
 *   ADC1 rank 3  IN5  (PB14)  NTC    47.5 cyc   1.86 - 3.27 us
 *   ADC2 rank 1  IN2  (PA1)   i_b     6.5 cyc   0.00 - 0.45 us
 *   ADC2 rank 2  IN17 (PA4)   V_bus  47.5 cyc   0.45 - 1.86 us
 * TRGO fires on the update event, which with CMS = 1 and RCR = 1 lands at
 * the centre of the high-side ON block. The ripple of a symmetric
 * centre-aligned pattern equals its own period average at that instant, so
 * rank 1 on both ADCs returns the AVERAGE phase current at the point
 * furthest from any switching edge. The cost is that at low duty the rank 2
 * and rank 3 windows straddle a real edge, which is why those channels are
 * judged statistically over a window rather than per sample. Rank 1 itself
 * moves closest to an edge when the duties hit the DUTY_MIN/DUTY_MAX clamp,
 * which is the case the OC confirmation counter of section 4b covers.
 *--------------------------------------------------------------------------*/
static void foc_adc_injected_config(void)
{
    ADC_InjectionConfTypeDef j = {0};
    HAL_StatusTypeDef s = HAL_OK;

    j.InjectedSingleDiff            = ADC_SINGLE_ENDED;
    j.InjectedOffsetNumber          = ADC_OFFSET_NONE;
    j.InjectedOffset                = 0;
    j.InjectedDiscontinuousConvMode = DISABLE;
    j.AutoInjectedConv              = DISABLE;
    j.QueueInjectedContext          = DISABLE;
    j.ExternalTrigInjecConv         = ADC_EXTERNALTRIGINJEC_T1_TRGO;
    j.ExternalTrigInjecConvEdge     = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    j.InjecOversamplingMode         = DISABLE;

    j.InjectedNbrOfConversion = 3;                      /* ADC1             */

    j.InjectedChannel      = ADC_CHANNEL_1;             /* PA0  i_a         */
    j.InjectedRank         = ADC_INJECTED_RANK_1;
    j.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &j) != HAL_OK) s = HAL_ERROR;

    j.InjectedChannel      = ADC_CHANNEL_15;            /* PB0  i_dc        */
    j.InjectedRank         = ADC_INJECTED_RANK_2;
    j.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &j) != HAL_OK) s = HAL_ERROR;

    j.InjectedChannel      = ADC_CHANNEL_5;             /* PB14 NTC         */
    j.InjectedRank         = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &j) != HAL_OK) s = HAL_ERROR;

    j.InjectedNbrOfConversion = 2;                      /* ADC2             */

    j.InjectedChannel      = ADC_CHANNEL_2;             /* PA1  i_b         */
    j.InjectedRank         = ADC_INJECTED_RANK_1;
    j.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &j) != HAL_OK) s = HAL_ERROR;

    j.InjectedChannel      = ADC_CHANNEL_17;            /* PA4  V_bus       */
    j.InjectedRank         = ADC_INJECTED_RANK_2;
    j.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &j) != HAL_OK) s = HAL_ERROR;

    if (s != HAL_OK) Error_Handler();
}

/* ---------------------------------------------------------------- gates ---*/
static void gates_enable(void)
{
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
}

static void gates_disable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    __HAL_TIM_MOE_DISABLE(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_ARR / 2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_ARR / 2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_ARR / 2);
    /* No voltage is being applied, so the observer must not be told one is:
       leaving the last command here parks a DC value in i_hat. */
    v_alpha = 0.0f;
    v_beta  = 0.0f;
}

static void foc_trip(uint8_t code)
{
    gates_disable();
    if (foc_state != FOC_FAULT) {
        fault_code = code;
        foc_state  = FOC_FAULT;
    }
#if TRACE_EN
    /* Freeze the recorder on the same event, keeping the pre-trigger
       history, so the capture shows the run-up to the trip rather than
       whatever happened afterwards. */
    trace_stop();
#endif
}

static void foc_reset(void)
{
    id_int = iq_int = 0;
    id_ref = iq_ref = 0;
    v_alpha = v_beta = 0;
    theta = theta_ol = 0;
    theta_off = 0;
    w_ol = 0;
    align_t = 0;
    ol_t = 0;                 /* or the soft fade works once per power cycle */
    ia_hat = ib_hat = 0;
    e_alpha = e_beta = 0;
    theta_trk = 0;
    w_est = 0;
    handoff_cnt   = 0;
    ho_wait       = 0;
    sync_lost_cnt = 0;
    idc_trip_cnt  = 0;
    oc_cnt        = 0;
    emf_bad_cnt   = 0;
    iq_floor      = 0.0f;
    iq_meas_f     = 0.0f;
    dbg_iq_floor  = 0.0f;
    dbg_iq_ref    = 0.0f;
    dbg_ho_best   = 10.0f;
#if TRACE_EN
    /* One capture per throttle-up. ANY PREVIOUS RECORD THAT WAS NOT DUMPED
       IS LOST HERE, which is deliberate: the interesting capture is almost
       always the most recent run. Dump before restarting. */
    trace_arm(TRACE_DECIM, TRACE_PRE);
#endif
    /* iq_max_eff and the iph_slow estimate in cap_limit_update() are
       deliberately NOT reset: they are thermal state, not control state. A
       restart takes a second and the capacitor cools over minutes, so
       clearing them per throttle cycle would let a rider reset the whole
       budget by feathering, which is the duty cycle that overheats it. */
}

/* --------------------------------------------------------- arm offset cal --
 * Boot, plus the normal-mode HASS retry from IDLE. Measures and commits the
 * HASS zero and range-checks the ACS averages; the phase offsets are the
 * constant ACS_OFF_FIXED and are never written. Suppressed in lab mode,
 * where the start is not gated on idc_sense_ok and it would re-enter
 * FOC_OFFSET_CAL once a second forever.
 * The published instantaneous currents are zeroed because the ISR returns
 * early throughout. The RMS values are deliberately NOT zeroed: they hold,
 * which keeps charging the capacitor thermal budget, and erring that way is
 * the whole design of cap_limit_update().
 *--------------------------------------------------------------------------*/
static void foc_cal_arm(void)
{
    off_acc_a = off_acc_b = off_acc_dc = 0.0f;
    off_n = 0;
    cal_settle = 0;
    ia = ib = ic = 0.0f;
    idc_a = 0.0f;
    foc_state = FOC_OFFSET_CAL;
}

/* ------------------------------------------------- leave a powered state ---
 * The only non-fault exit from ALIGN, OPEN_LOOP or CLOSED_LOOP. Gates off,
 * then stamp the earliest tick at which ALIGN may be re-entered. Callers
 * must not write foc_state afterwards and must goto isr_exit, because the
 * local st in the ISR still holds the old state.
 * The recorder is deliberately left RUNNING here: a coast-down after a
 * handoff give-up is often the part worth looking at, and the next
 * throttle-up re-arms it anyway.
 *--------------------------------------------------------------------------*/
static void foc_to_idle(uint32_t hold_ms)
{
    gates_disable();
    restart_block_until = HAL_GetTick() + hold_ms;
    ia = ib = ic = 0.0f;
    idc_a = 0.0f;
    foc_state = FOC_IDLE;
}

/* ------------------------------------ DC link capacitor thermal ceiling ----
 * Called once per 100 ms pass, after the RMS window is reduced. An AVERAGE,
 * not a peak test, because the can's thermal constant is minutes: a ten
 * second takeoff burst does not heat it, sustained RMS does. A peak limiter
 * would cut thrust exactly when the rider needs it and still permit an
 * overheating duty cycle of moderate current. Scaling is linear in CURRENT,
 * not power: halving current quarters dissipation, so a linear rollback is
 * conservative and converges quickly.
 *--------------------------------------------------------------------------*/
static void cap_limit_update(void)
{
    static float iph_slow = 0.0f;

    /* The three phases sum to zero, so this is the true per-phase RMS in
       steady state. */
    float iph = sqrtf((ia_rms * ia_rms + ib_rms * ib_rms + ic_rms * ic_rms)
                      * 0.3333333f);

    float tau = (iph > iph_slow) ? TAU_CAP_RISE_S : TAU_CAP_FALL_S;
    iph_slow += (0.1f / tau) * (iph - iph_slow);
    if (iph_slow < 0.0f) iph_slow = 0.0f;
    dbg_iph_slow = iph_slow;

    float scale = CLAMPF(IPH_RMS_CONT_A / fmaxf(iph_slow, 1.0f), 0.0f, 1.0f);
    float lim   = fmaxf(IQ_MAX_A * scale, IQ_MAX_FLOOR_A);

    iq_max_eff     = fminf(lim, IQ_MAX_A);
    dbg_iq_max_eff = iq_max_eff;
}

/* ------------------------------------------------------------ FOC ISR ----
 * 20 kHz, from the ADC1 injected end-of-sequence interrupt. ADC1 JDR1 and
 * ADC2 JDR1 were sampled simultaneously at the TIM1 TRGO instant; JEOS lands
 * about 3.3 us later once ranks 2 and 3 have converted, so every JDR is
 * fresh.
 *--------------------------------------------------------------------------*/
static void foc_isr(void)
{
    uint32_t cyc0 = DWT->CYCCNT;

    /* ---- 1. raw acquisition ---------------------------------------------- */
    uint32_t rawi_a  = ADC1->JDR1;
    uint32_t rawi_b  = ADC2->JDR1;
    uint32_t rawi_dc = ADC1->JDR2;
    uint32_t rawntc  = ADC1->JDR3;
    uint32_t rawvb   = ADC2->JDR2;

    /* Snapshot shared state ONCE. vdc comes from the FAST bus estimate, not
       from the 100 ms window mean: the modulator, the voltage circle, the
       dead-time voltage and the observer voltage model all divide by it, and
       a supply entering constant current moves the real bus in about a
       millisecond. The window mean still owns the FAULT decision. */
    foc_state_t st     = foc_state;
    float vb_now = (float)rawvb * (VREF / ADC_FS) * VBUS_GAIN;
    vbus_fast += VBUS_FAST_ALPHA * (vb_now - vbus_fast);
    float vdc = vbus_sense_fault ? VBUS_FALLBACK
                                 : CLAMPF(vbus_fast, VBUS_PLAUS_MIN, VBUS_PLAUS_MAX);
    float       iq_lim = iq_max_eff;

    /* ---- 1b. slow-channel window statistics ------------------------------ */
    if (rawntc >= (uint32_t)NTC_RAW_HI || rawntc <= (uint32_t)NTC_RAW_LO) {
        ntc_acc_bad++;
    } else {
        ntc_acc_sum += rawntc;
        ntc_acc_good++;
    }
    vb_acc_sum += rawvb;
    vb_acc_n++;
    if (rawvb >= VBUS_RAW_CLIP) vb_acc_clip++;

    /* ---- 1c. boot offset calibration -------------------------------------
     * The first CAL_SETTLE_N ticks are discarded so nothing from the
     * power-up transient lands in the average. */
    if (st == FOC_OFFSET_CAL) {
        if (cal_settle < CAL_SETTLE_N) {
            cal_settle++;
            goto isr_exit;
        }

        off_acc_a  += (float)rawi_a;
        off_acc_b  += (float)rawi_b;
        off_acc_dc += (float)rawi_dc;
        if (++off_n >= CAL_N) {
            float a_new  = off_acc_a  / (float)off_n;
            float b_new  = off_acc_b  / (float)off_n;
            float dc_new = off_acc_dc / (float)off_n;

            /* A dead ACS produces a garbage average and then an OC trip one
               tick later, which reads as an overcurrent instead of a wiring
               fault. ARMED IN LAB MODE TOO: without valid phase current the
               FOC runs on garbage and the phase OC trip, the only protection
               lab mode leaves, measures nothing. */
            if (a_new < ACS_OFF_MIN || a_new > ACS_OFF_MAX ||
                b_new < ACS_OFF_MIN || b_new > ACS_OFF_MAX) {
                foc_trip(FAULT_ACS_SAT);
            } else {
                dbg_ia_off     = a_new;
                dbg_ib_off     = b_new;
                dbg_ia_drift_a = (a_new - ACS_OFF_FIXED) * ADC_A_PER_COUNT;
                dbg_ib_drift_a = (b_new - ACS_OFF_FIXED) * ADC_A_PER_COUNT;
                dbg_cal_done++;

                /* The HASS is the only DC link protection there is, so an
                   implausible zero blocks the start rather than silently
                   removing it. Not latched: cal is re-armed from IDLE once
                   the bus, and therefore the 5 V rail, appears. */
                idc_off      = dc_new;
                dbg_idc_off  = idc_off;
                idc_sense_ok = (idc_off > HASS_OFF_MIN &&
                                idc_off < HASS_OFF_MAX) ? 1u : 0u;
                foc_state = FOC_IDLE;
            }
        }
        goto isr_exit;
    }

    /* ---- 2. scale to amps ------------------------------------------------
     * No mid-run raw-count guard, deliberately: a rail at raw 4065 scales to
     * 246 A and raw 30 to -246 A, both past OC_TRIP_A, so the guard was
     * shadowed in every reachable case and only risked relabelling a
     * shoot-through as a sensor fault. */
    float la = IA_SIGN * ((float)rawi_a - ia_off) * (VREF / ADC_FS) * ACS_A_PER_V;
    float lb = IB_SIGN * ((float)rawi_b - ib_off) * (VREF / ADC_FS) * ACS_A_PER_V;
    float lc = -(la + lb);
    ia = la; ib = lb; ic = lc;

    float ldc = HASS_SIGN * ((float)rawi_dc - idc_off) * (VREF / ADC_FS)
                * HASS_A_PER_V;
    idc_a = ldc;

    /* ---- 3. protection ---------------------------------------------------
     * The phase trip is armed in BOTH modes: it is the only thing between a
     * wiring error and a destroyed module, it runs off the clean rank-1
     * samples, and it is faster than any bench supply's current limit. It is
     * only as good as la/lb/lc, which is why dbg_vd exists.
     * CONFIRMED OVER OC_CONFIRM_N SAMPLES, see section 4b. Against 40 uH the
     * phase current cannot move more than about 70 A in one 50 us tick, so a
     * lone sample past 190 A is a switching-edge artefact rather than a
     * current. Two consecutive samples cost 100 us of delay on a module
     * rated 300 A RMS. dbg_oc_singles counts the rejected ones: if it climbs
     * during normal running, the rank-1 sample is picking up switching noise
     * and that, not the drive, is what needs fixing next. */
    if (fabsf(la) > OC_TRIP_A || fabsf(lb) > OC_TRIP_A || fabsf(lc) > OC_TRIP_A) {
        if (++oc_cnt >= (uint16_t)OC_CONFIRM_N) {
            foc_trip(FAULT_PHASE_OC);
            goto isr_exit;
        }
    } else {
        if (oc_cnt) dbg_oc_singles++;
        oc_cnt = 0;
    }
    if (fabsf(ldc) > IDC_TRIP_A) {
        if (++idc_trip_cnt >= IDC_TRIP_N) {
            if (!LAB_TEST_MODE) {
            	foc_trip(FAULT_DCLINK_OC);
            	goto isr_exit;
            }
            idc_trip_cnt = IDC_TRIP_N;   /* held so it cannot wrap           */
        }
    } else {
        idc_trip_cnt = 0;
    }

    /* ---- 3b. phase current sum of squares, after the trips so it runs in
     * IDLE too and an idle drive reports a true 0 A rather than a stale
     * number. */
    irms_acc_a += la * la;
    irms_acc_b += lb * lb;
    irms_acc_c += lc * lc;
    irms_acc_n++;

    /* ---- 4. Clarke, amplitude invariant ---------------------------------- */
    float i_alpha = la;
    float i_beta  = (la + 2.0f * lb) * 0.5773503f;      /* 1/sqrt(3)         */

    uint8_t powered = (st == FOC_ALIGN || st == FOC_OPEN_LOOP ||
                       st == FOC_CLOSED_LOOP);
    float theta_est = 0.0f;

    if (powered) {
        /* ---- 5. SLIDING MODE OBSERVER, equations (3) to (11) -------------
         * s = i_hat - i is the sliding variable; the injection is a true
         * sign function, so the manifold is reached in finite time provided
         * the adaptive gain satisfies the existence condition (5). */
        float s_a = ia_hat - i_alpha;
        float s_b = ib_hat - i_beta;

        float ksw = SMO_K0 + SMO_K_MARGIN * FLUX_WB * w_est;   /* eq. (6)   */
        if (ksw > SMO_K_MAX) ksw = SMO_K_MAX;
        dbg_ksw = ksw;

        float za = (s_a >= 0.0f) ? ksw : -ksw;
        float zb = (s_b >= 0.0f) ? ksw : -ksw;

        /* Forward Euler on (3). v_alpha/v_beta are the reconstructed ACTUAL
           terminal voltages from the previous tick, which is the voltage
           applied over the period this current sample averages. */
        ia_hat += TS * (v_alpha - RS_OHM * ia_hat - za) * (1.0f / LS_H);
        ib_hat += TS * (v_beta  - RS_OHM * ib_hat - zb) * (1.0f / LS_H);

        dbg_smo_chat = 0.5f * (fabsf(s_a) + fabsf(s_b));   /* band, eq. (11)*/

        /* ---- 6. equivalent control extraction, equations (7) to (10) -----
         * One real pole per axis at wc = wc0 + w_hat. Because both axes are
         * filtered identically, the EMF VECTOR is simply rotated by -atan(r)
         * and scaled by 1/sqrt(1+r^2). Only the ROTATION matters to the
         * angle, and it is inverted exactly in (13). */
        float wc    = SMO_WC0 + w_est;
        float k_lpf = TS * wc;                  /* < 1 by construction      */
        e_alpha += k_lpf * (za - e_alpha);
        e_beta  += k_lpf * (zb - e_beta);

        /* ---- 7. CORDIC argument and ANGLE TRACKING OBSERVER --------------
         * One vectoring-mode transaction returns both the argument (12a) and
         * the modulus of the filtered EMF vector. The argument drives the
         * tracking observer, which is the ONLY source of speed. The modulus
         * is used for the validity gate and for the flux diagnostic (14),
         * and for nothing else. */
        float emag;
        float th_raw = cordic_phase(-e_alpha, e_beta, &emag);
        float r_lpf  = w_est / wc;              /* r at the filter of (8)   */

        if (emag > EMF_MIN_V) {
            /* Type-2 tracking loop, equation (12b). The wrap makes the error
               continuous across +/-pi; the clamp bounds the acquisition kick
               when the loop is released from the I-f seed. */
            float eps = th_raw - theta_trk;
            WRAP_2PI(eps);
            eps = CLAMPF(eps, -ATO_EPS_LIM, ATO_EPS_LIM);
            dbg_ato_eps = eps;

            w_est += ATO_KI * eps * TS;
            /* FORWARD ONLY. The EMF vector cannot tell (theta, +w) from
               (theta+pi, -w); denying the integrator negative values removes
               the reverse lock, where +iq_ref would make -iq and throttle
               would drive the propeller backwards harder. A genuinely
               reversed rotor pins w_est at 0, which is below W_SYNC_MIN and
               trips the existing sync-lost path. */
            w_est  = CLAMPF(w_est, 0.0f, W_EST_MAX);

            theta_trk += (w_est + ATO_KP * eps) * TS;
            WRAP_2PI(theta_trk);

            /* Equation (14). A REAL measurement now that w_est does not
               depend on FLUX_WB: the bench check on V_DEV_A and on the KV
               convention. */
            if (w_est > W_SYNC_MIN) {
                dbg_lambda_est = emag * sqrtf(1.0f + r_lpf * r_lpf) / w_est;
            }
            emf_bad_cnt = 0;
        } else if (st == FOC_OPEN_LOOP) {
            /* Below EMF_MIN_V the argument of (12a) is noise. Seed the
               tracker from the forced angle rather than freezing it, so it
               is released from approximately the right place with most of
               the ramp left to converge. */
            theta_trk   = theta_ol;
            w_est       = w_ol;
            dbg_ato_eps = 0.0f;
            handoff_cnt = 0;                    /* it agrees with itself    */
            emf_bad_cnt = 0;
        } else {
            /* Closed loop with an unusable estimate: COAST on the last
               speed. Forcing w_est to zero would freeze theta_trk while the
               rotor kept turning, the tracking error would grow without
               bound and one noise dip would guarantee a sync-lost trip on a
               healthy machine. emf_bad_cnt owns that decision instead. */
            theta_trk += w_est * TS;
            WRAP_2PI(theta_trk);
            if (emf_bad_cnt <= (uint16_t)SYNC_LOST_N) emf_bad_cnt++;
            if (emf_bad_cnt > dbg_emf_bad_max) dbg_emf_bad_max = emf_bad_cnt;
        }

        /* ---- Equation (13). Both corrections are applied to the SMOOTH
         * tracker output, outside the loop, so correction noise is not fed
         * back into it. r is recomputed from the updated w_est and is < 1 by
         * construction, which is the CORDIC arctan domain. */
        float r_corr = w_est / (SMO_WC0 + w_est);
        theta_est = theta_trk + cordic_atan(r_corr) + w_est * SMO_DELAY_S;
        WRAP_2PI(theta_est);
    }

    /* ---- 7b. speed accumulator for the RPM telemetry field ---------------
     * Same window as the RMS currents. Reports the commanded ramp in open
     * loop, the tracking observer in closed loop, and a true zero elsewhere,
     * rather than a stale estimate. */
    {
        float w_tel = (st == FOC_CLOSED_LOOP) ? w_est
                    : ((st == FOC_OPEN_LOOP)  ? w_ol : 0.0f);
        w_acc += w_tel;
        w_acc_n++;
    }

    /* ---- 8. state machine ------------------------------------------------ */
    uint32_t now  = HAL_GetTick();
    uint8_t  live = ((now - last_valid_rx_tick) < BLE_TIMEOUT_MS) ? 1u : 0u;
    float    thr  = live ? throttle_percent : 0.0f;

    switch (st)
    {
    case FOC_IDLE:
    {
        /* restart_block_until was stamped by foc_to_idle() at the instant the
           gates opened; the comparison is signed, so the 49-day tick wrap
           cannot unblock or permanently block a start. A rider holding the
           throttle through a failure WILL get an automatic restart once the
           hold expires: that is deliberate, it is how you recover after
           dropping off foil, and it is why the hold is 1 s and not 50 ms. */
        uint8_t idc_gate = LAB_TEST_MODE ? 1u : idc_sense_ok;
        if (thr > THR_START_PCT && !vbus_sense_fault && idc_gate &&
            TICK_REACHED(now, restart_block_until)) {
            foc_reset();
            gates_enable();
            foc_state = FOC_ALIGN;
        }
        goto isr_exit;
    }

    case FOC_ALIGN:
        /* Stationary, known DC current at a known angle, no back-EMF, so the
           required voltage is a hand calculation: vd = I_ALIGN_A*Rs + Vdt =
           2.5 V against dbg_vmax = 15 V. Check this before anything
           downstream. */
        theta = 0.0f;
        align_t += TS;
        id_ref = I_ALIGN_A * CLAMPF(align_t / (0.5f * ALIGN_TIME_S), 0.0f, 1.0f);
        iq_ref = 0.0f;
        if (align_t >= ALIGN_TIME_S) {
            theta_ol  = 0.0f;
            w_ol      = 0.0f;
            ol_t      = 0.0f;
            theta_trk = 0.0f;
            w_est     = 0.0f;
            ho_wait   = 0;
#if TRACE_EN
            /* t = 0 of the capture is the start of the I-f ramp, with
               TRACE_PRE samples of ALIGN kept before it. */
            trace_trigger();
#endif
            foc_state = FOC_OPEN_LOOP;
        }
        break;

    case FOC_OPEN_LOOP:
    {
        /* Speed ramp, HELD at W_HANDOFF rather than allowed to run away, so
           the acquisition attempt happens at one fixed operating point and
           HO_WAIT_N bounds it. */
        float w = w_ol + OL_ACCEL * TS;
        if (w > W_HANDOFF) w = W_HANDOFF;
        w_ol = w;

        /* I-f damping injection. Gated on a usable speed estimate, and
           exactly zero while the tracker is being seeded, because the seed
           sets w_est = w_ol. */
        float w_dmp = 0.0f;
        if (w_est > W_SYNC_MIN) {
            w_dmp = CLAMPF(OL_DAMP_K * (w_est - w),
                           -OL_DAMP_LIM * w, OL_DAMP_LIM * w);
        }
        theta_ol += (w + w_dmp) * TS;
        WRAP_2PI(theta_ol);
        theta = theta_ol;

        /* Current ramps with speed so torque is available as propeller load
           rises with the square of speed. The soft factor multiplies the
           WHOLE expression: fading only the constant term would leave the
           speed-dependent part stepping in at full value. */
        ol_t += TS;
        float soft  = CLAMPF(ol_t * (1.0f / OL_SOFT_S), 0.0f, 1.0f);
        float i_tgt = I_STARTUP_A0 + (I_STARTUP_A1 - I_STARTUP_A0)
                      * CLAMPF(w / W_HANDOFF, 0.0f, 1.0f);
        id_ref = I_ALIGN_A + soft * (i_tgt - I_ALIGN_A);
        iq_ref = 0.0f;

        if (w >= W_HANDOFF) {
            float derr = theta_est - theta_ol;
            WRAP_2PI(derr);
            dbg_ho_derr = derr;
            if (fabsf(derr) < dbg_ho_best) dbg_ho_best = fabsf(derr);

            if (fabsf(derr) < HANDOFF_ERR_RAD &&
                fabsf(w_est - w) < HANDOFF_W_FRAC * w) {
                if (++handoff_cnt >= HANDOFF_HOLD_N) {
                    theta_off = derr;
                    /* The I-f vector is i_tgt on the theta_ol axis and the
                       rotor lags it, so its torque-producing component in
                       the rotor frame is -i_tgt*sin(derr). Seeding iq_ref
                       there, and slewing id_ref down instead of stepping it,
                       removes the torque gap at the transition. */
                    iq_ref   = CLAMPF(-i_tgt * sinf(derr), 0.0f, iq_lim);
                    iq_floor = fmaxf(iq_ref, IQ_HANDOFF_FRAC * i_tgt);
                    /* Seed the anti-windup filter with the current actually
                       flowing, so the cap does not clamp iq_ref to zero on
                       the first closed-loop tick. */
                    iq_meas_f     = iq_meas;
                    emf_bad_cnt   = 0;
                    sync_lost_cnt = 0;
                    ho_wait       = 0;
                    foc_state = FOC_CLOSED_LOOP;
                }
            } else if (handoff_cnt > HANDOFF_DECAY) {
                /* DECAY, NOT RESET: a single noisy tick used to throw away
                   the whole 5 ms of agreement, which on a machine ringing at
                   5.6 Hz meant the counter never finished. */
                handoff_cnt -= HANDOFF_DECAY;
            } else {
                handoff_cnt = 0;
            }

            /* Give-up timer. If it fires, read dbg_ho_best: close to
               HANDOFF_ERR_RAD means the gate or the hold count, far from it
               means Ls, V_DEV_A or the dead-time model. */
            if (foc_state == FOC_OPEN_LOOP) {
                if (++ho_wait > (uint16_t)HO_WAIT_N) {
                    ho_wait = 0;
                    dbg_ho_fails++;
                    foc_to_idle(FAIL_HOLD_MS);
                    goto isr_exit;
                }
            }
        } else {
            ho_wait = 0;
        }

        if (thr < THR_STOP_PCT) {
            foc_to_idle(RESTART_HOLD_MS);
            goto isr_exit;
        }
        break;
    }

    case FOC_CLOSED_LOOP:
    {
        theta_off *= (1.0f - TS / BLEND_TAU_S);
        theta = theta_est - theta_off;
        WRAP_2PI(theta);

        /* d axis slews down from the I-f value rather than stepping. There
           must be no id_ref = 0.0f above this or it is dead code. */
        float id_step = ID_SLEW_A_S * TS;
        if      (id_ref >  id_step) id_ref -= id_step;
        else if (id_ref < -id_step) id_ref += id_step;
        else                        id_ref  = 0.0f;

        /* Post-handoff torque hold, killed outright below THR_STOP_PCT so it
           can never hold torque against the dead-man. */
        if (thr < THR_STOP_PCT) {
            iq_floor = 0.0f;
        } else {
            iq_floor -= iq_floor * (TS / IQ_HANDOFF_HOLD_S);
            if (iq_floor < 0.01f) iq_floor = 0.0f;
        }
        dbg_iq_floor = iq_floor;

        float target = fmaxf((thr / 100.0f) * iq_lim, fminf(iq_floor, iq_lim));
        float step   = IQ_SLEW_A_S * TS;
        if      (iq_ref < target - step) iq_ref += step;
        else if (iq_ref > target + step) iq_ref -= step;
        else                             iq_ref  = target;
        iq_ref = CLAMPF(iq_ref, -iq_lim, iq_lim);

        /* ---- REFERENCE ANTI-WINDUP, section 4b --------------------------
           On the voltage circle the q PI has no authority left, so an
           unreachable iq_ref only parks the integrator on the limit and
           leaves the phase current set by angle error alone at roughly
           300 A per radian. Hold iq_ref within IQ_SAT_MARGIN_A of the
           current actually flowing whenever the q axis is saturated. This
           does not lower the top speed, which is set by the voltage circle
           and not by the reference; it keeps the loop in the same regime it
           occupies at part throttle.
           iq_meas and dbg_v_sat are from the previous tick, i.e. one 50 us
           delay on a limit that moves at the mechanical rate. dbg_v_sat is
           therefore LOAD-BEARING: do not remove it with the debug block. */
        iq_meas_f += 0.05f * (iq_meas - iq_meas_f);        /* 1 ms IIR      */
        if (dbg_v_sat & 2u) {
            float iq_cap = fmaxf(iq_meas_f + IQ_SAT_MARGIN_A, 0.0f);
            if (iq_ref > iq_cap) iq_ref = iq_cap;
        }
        dbg_iq_ref = iq_ref;

        /* Normal stop on release. The emf_bad_cnt term is required because
           w_est coasts: without it, a machine that lost its EMF estimate
           with the throttle released would hold a stale w_est above the
           threshold and never shut the gates down. */
        if (thr < THR_STOP_PCT &&
            (w_est < 0.5f * W_HANDOFF ||
             emf_bad_cnt >= (uint16_t)SYNC_LOST_N)) {
            foc_to_idle(RESTART_HOLD_MS);
            goto isr_exit;
        }

        /* Lost sync, gated on the throttle asking for torque so a normal
           coast-down does not look like a dropout. Two independent causes:
           the machine stalled (w_est below W_SYNC_MIN) or the observer
           stopped producing a usable estimate (emf_bad_cnt). */
        if (thr > THR_STOP_PCT &&
            (w_est < W_SYNC_MIN || emf_bad_cnt >= (uint16_t)SYNC_LOST_N)) {
            if (++sync_lost_cnt >= SYNC_LOST_N) {
                sync_lost_cnt = 0;
                dbg_sync_exits++;
                foc_to_idle(FAIL_HOLD_MS);
                goto isr_exit;
            }
        } else {
            sync_lost_cnt = 0;
        }
        break;
    }

    case FOC_FAULT:
    default:
        gates_disable();
        goto isr_exit;
    }

    /* ---- 9. Park, CORDIC rotation mode ----------------------------------- */
    float s, c;
    cordic_sincos(theta, &s, &c);
    float id =  i_alpha * c + i_beta * s;
    float iq = -i_alpha * s + i_beta * c;
    id_meas = id;
    iq_meas = iq;

    /* ---- 10. current PI with anti-windup ---------------------------------
     * The d axis has priority in the voltage circle, so the q INTEGRATOR is
     * clamped to the same limit as vq; clamping it to vmax while vq was
     * clamped to vq_lim let it wind up outside the circle whenever the d
     * axis took voltage.
     * BACK-CALCULATION on both axes: the integrator is advanced, the output
     * formed and clamped, then whatever the clamp removed is subtracted back
     * out, so the integrator settles exactly on the limit and releases the
     * instant the error changes sign. A bare clamp parks it AT the limit
     * with no memory of how far past it went.
     * This is INTEGRATOR anti-windup. It bounds the integrator but not the
     * reference, which is why section 4b bounds iq_ref separately. */
    float vmax = VMAX_FRAC * vdc;
    if (st == FOC_ALIGN || st == FOC_OPEN_LOOP) vmax = fminf(vmax, VMAX_START_V);

    /* The d axis may not take the whole circle outside closed loop: in ALIGN
       and the I-f ramp id_ref is the only current command, so an unreachable
       id_ref would wind id_int to vmax, collapse vq_lim to zero and starve
       the q axis against the back-EMF. */
    float vd_lim = (st == FOC_CLOSED_LOOP) ? vmax : (0.5f * vmax);

    float ed = id_ref - id;
    float eq = iq_ref - iq;

    id_int += KI_I * ed * TS;
    float vd_unsat = KP_I * ed + id_int;
    float vd = CLAMPF(vd_unsat, -vd_lim, vd_lim);
    id_int -= 0.3f * (vd_unsat - vd);
    id_int  = CLAMPF(id_int, -vd_lim, vd_lim);

    float vq_lim = sqrtf(fmaxf(vmax * vmax - vd * vd, 0.0f));

    iq_int += KI_I * eq * TS;
    float vq_unsat = KP_I * eq + iq_int;
    float vq = CLAMPF(vq_unsat, -vq_lim, vq_lim);
    iq_int -= 0.3f * (vq_unsat - vq);
    iq_int  = CLAMPF(iq_int, -vq_lim, vq_lim);

    dbg_vmax  = vmax;
    dbg_vd    = vd;
    dbg_vq    = vq;
    dbg_v_sat = (uint8_t)((fabsf(vd) >= 0.999f * vd_lim ? 1u : 0u) |
                          ((vq_lim > 0.01f && fabsf(vq) >= 0.999f * vq_lim)
                           ? 2u : 0u));

    /* ---- 11. inverse Park ------------------------------------------------ */
    float va_cmd = vd * c - vq * s;
    float vb_cmd = vd * s + vq * c;

    /* ---- 12. dead-time compensation, SVPWM, observer voltage -------------*/
    {
        float vdt     = vdc * (DEADTIME_S / TS);   /* about 2.0 V at 50.4 V */
        float vdt_obs = vdt + V_DEV_A;             /* observer model only   */

        float va = va_cmd;
        float vb = -0.5f * va_cmd + 0.8660254f * vb_cmd;
        float vc = -0.5f * va_cmd - 0.8660254f * vb_cmd;

        float sa = CLAMPF(la * (1.0f / DT_LIN_A), -1.0f, 1.0f);
        float sb = CLAMPF(lb * (1.0f / DT_LIN_A), -1.0f, 1.0f);
        float sc = CLAMPF(lc * (1.0f / DT_LIN_A), -1.0f, 1.0f);
        float oa = CLAMPF(la * (1.0f / DT_OBS_A), -1.0f, 1.0f);
        float ob = CLAMPF(lb * (1.0f / DT_OBS_A), -1.0f, 1.0f);
        float oc = CLAMPF(lc * (1.0f / DT_OBS_A), -1.0f, 1.0f);

#if DT_COMP_EN
        va += vdt * sa;
        vb += vdt * sb;
        vc += vdt * sc;
#endif

        float vmx = fmaxf(va, fmaxf(vb, vc));
        float vmn = fminf(va, fminf(vb, vc));
        float vcm = 0.5f * (vmx + vmn);

        float inv_vdc = 1.0f / vdc;
        float da = CLAMPF(0.5f + (va - vcm) * inv_vdc, DUTY_MIN, DUTY_MAX);
        float db = CLAMPF(0.5f + (vb - vcm) * inv_vdc, DUTY_MIN, DUTY_MAX);
        float dc = CLAMPF(0.5f + (vc - vcm) * inv_vdc, DUTY_MIN, DUTY_MAX);

        float dcm = (da + db + dc) * 0.3333333f;
        float ra  = (da - dcm) * vdc - vdt_obs * oa;
        float rb  = (db - dcm) * vdc - vdt_obs * ob;
        float rc  = (dc - dcm) * vdc - vdt_obs * oc;
        v_alpha = (2.0f * ra - rb - rc) * 0.3333333f;
        v_beta  = (rb - rc) * 0.5773503f;

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(da * PWM_ARR));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(db * PWM_ARR));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dc * PWM_ARR));
        dbg_duty_a = da;
    }

#if TRACE_EN
    /* ---- 13. trace record, section 5 -------------------------------------
     * MUST STAY ON THIS SIDE OF isr_exit. Several of the goto paths above
     * jump over the initialisation of i_alpha, theta_est, id and iq, so
     * sampling them at the label would read indeterminate values. The paths
     * that skip this either apply no voltage (IDLE, OFFSET_CAL) or have
     * already frozen the buffer through foc_trip().
     * The order here must match trace_ch[] exactly.
     * iq_ref is logged AFTER the state machine has applied the slew limiter,
     * iq_max_eff and the section 4b cap, so it is the command the drive
     * actually acted on, not the raw throttle scaling. In ALIGN and
     * OPEN_LOOP it is zero by construction, which is correct: all the
     * current is on the d axis there.
     * Cost: one sqrtf (about 25 cycles with the FPU VSQRT) plus nine
     * channels of accumulate and store, roughly 180 of the 8500 cycle
     * budget. Compare dbg_isr_cycles_max before and after rather than
     * trusting that number. */
    {
        float i_s = sqrtf(i_alpha * i_alpha + i_beta * i_beta);
        const float tr_smp[TRACE_CH_N] = {
            w_est * RPM_PER_WE,     /* tracking observer, mechanical rpm   */
            w_ol  * RPM_PER_WE,     /* commanded I-f ramp, mechanical rpm  */
            id,                     /* measured d-axis current             */
            iq,                     /* measured q-axis current             */
            iq_ref,                 /* torque command after every limiter  */
            i_s,                    /* peak phase current, angle-blind     */
            vd,                     /* d-axis voltage command, post-clamp  */
            vq,                     /* q-axis voltage command, post-clamp  */
            ldc,                    /* DC link current, this tick          */
            vbus_fast,              /* 1 ms bus estimate the ISR runs on   */
            module_temp_c,          /* module NTC, slow loop value         */
            (float)st               /* state machine, see trace_ch[]       */
        };
        trace_put(tr_smp);
    }
#endif

isr_exit:
    dbg_isr_cycles = DWT->CYCCNT - cyc0;
    if (dbg_isr_cycles > dbg_isr_cycles_max) dbg_isr_cycles_max = dbg_isr_cycles;
}

/* ---------------------------------------------------------------- NTC ----
 * Pure function, beta equation only. The caller owns the range check and the
 * fault decision: a fault declared here could not be debounced and a
 * sentinel returned here would poison the caller's IIR.
 *   R_NTC = R_FIXED*raw/(FS - raw),  1/T = 1/T25 + (1/B) ln(R_ntc/R25)
 *--------------------------------------------------------------------------*/
static float ntc_temp_c(uint16_t raw)
{
    float r = NTC_R_FIXED * (float)raw / (ADC_FS - (float)raw);
    float invT = 1.0f / NTC_T25_K + (1.0f / NTC_BETA) * logf(r / NTC_R25);
    return (1.0f / invT) - 273.15f;
}

/* -------------------------------------------------- BLE throttle parse ----*/
static void parse_ble_bytes(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        char ch = (char)buf[i];
        if (ch == '\n') {
            line_buf[line_idx] = '\0';
            if (line_buf[0] == 'T' && line_idx >= 2) {
                float v = strtof(&line_buf[1], NULL);
                if (v >= 0.0f && v <= 100.0f) {
                    throttle_percent   = v;
                    last_valid_rx_tick = HAL_GetTick();
                }
            }
            line_idx = 0;
        } else if (line_idx < sizeof(line_buf) - 1) {
            line_buf[line_idx++] = ch;
        } else {
            line_idx = 0;                          /* garbage, resync       */
        }
    }
}

/* ------------------------------------------------- BLE DMA re-arm ---------
 * HAL_UARTEx_ReceiveToIdle_DMA enables the half-transfer interrupt every
 * time it is called, which with a 64-byte buffer and short packets gives a
 * spurious RxEvent at 32 bytes with a partial line. Suppress HT immediately
 * after each arm, but ONLY when the arm succeeded: the call can return
 * HAL_BUSY at its own lock, and discarding that makes a failed arm silent
 * and permanent.
 *--------------------------------------------------------------------------*/
static void ble_rx_arm(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ble_rx_buf, BLE_RX_BUF_LEN)
        == HAL_OK) {
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

/* A dead receiver takes the throttle, the dead-man and the fault-clear path
   with it, so a missed arm must not be permanent. Re-arming an already-idle
   receiver is harmless. */
static void ble_rx_watchdog(void)
{
    if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {
        dbg_rx_rearm++;
        ble_rx_arm();
    }
}

/* ------------------------------------------------------ telemetry TX ------
 * telem_uart_ready() is the stuck-transmitter recovery: if gState stays busy
 * for TELEM_STUCK_MS the transfer is aborted and forced back to READY,
 * because going permanently silent is the failure being guarded against.
 * There is deliberately NO blocking path: 38 bytes at 9600 baud is 40 ms of
 * stalled main loop, which would take the RX watchdog and the dead-man with
 * it.
 *--------------------------------------------------------------------------*/
static uint8_t telem_uart_ready(void)
{
    static uint32_t busy_since = 0;

    if (huart1.gState != HAL_UART_STATE_READY) {
        dbg_telem_skipped++;
        if (busy_since == 0) {
            busy_since = HAL_GetTick();
        } else if (HAL_GetTick() - busy_since > TELEM_STUCK_MS) {
            HAL_UART_AbortTransmit(&huart1);
            huart1.gState = HAL_UART_STATE_READY;
            busy_since = 0;
        }
        return 0u;
    }
    busy_since = 0;
    return 1u;
}

static void telem_queue(int n)
{
    HAL_StatusTypeDef stat;

    if (n <= 0 || n > (int)sizeof(telem_tx) - 1) {
        dbg_telem_skipped++;          /* truncated frames fail the remote's
                                         field-count check anyway           */
        return;
    }

    if (huart1.hdmatx != NULL) {
        stat = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)telem_tx, (uint16_t)n);
    } else {
        stat = HAL_UART_Transmit_IT(&huart1, (uint8_t *)telem_tx, (uint16_t)n);
    }

    if (stat == HAL_OK) dbg_telem_sent++;
    else                dbg_telem_skipped++;
}

/* Fixed-point pack for the debug frame. The clamp keeps every field to five
   characters plus sign, so the worst-case frame is a known 32 bytes. */
static int telem_scale_i(float v, float k)
{
    int x = (int)(v * k);
    if (x >  9990) x =  9990;
    if (x < -9990) x = -9990;
    return x;
}

/* --------------------------------------------- status frame ('S' / 'L') ---
 * NINE FIELDS, FIXED. Only the lead character changes with lab test mode.
 * State and fault must each stay ONE digit: the remote indexes them as sf[0]
 * and sf[1] of a single field, so a two-digit state would shift every field
 * after it and the whole frame would be discarded.
 *--------------------------------------------------------------------------*/
static void telem_send(void)
{
    if (!telem_uart_ready()) return;

    int t10;
    if (ntc_fault) {
        t10 = TELEM_T10_FAULT;
    } else {
        t10 = (int)(module_temp_c * 10.0f);
        t10 = (t10 >  9990) ?  9990 : t10;
        t10 = (t10 < -9990) ? -9990 : t10;
    }

    int vb10 = vbus_sense_fault ? -1 : (int)(vbus_v * 10.0f);

    int idc_tx;
    if (idc_sense_ok) {
        idc_tx = (int)idc_a;
        if (idc_tx < -9000) idc_tx = -9000;
        if (idc_tx >  9000) idc_tx =  9000;
    } else {
        idc_tx = TELEM_IDC_ABSENT;
    }

    int rpm = (int)motor_rpm;
    rpm = (rpm < 0) ? 0 : ((rpm > 9999) ? 9999 : rpm);

    unsigned stt = (unsigned)foc_state;
    unsigned flt = (unsigned)fault_code;
    if (stt > 5u) stt = 5u;
    if (flt > 4u) flt = 4u;

    int n = snprintf(telem_tx, sizeof(telem_tx),
                     "%c,%u%u,%d,%d,%d,%d,%d,%d,%d\n",
                     (char)TELEM_LEAD_CH,
                     stt, flt, t10, rpm, idc_tx, vb10,
                     (int)(ia_rms + 0.5f),
                     (int)(ib_rms + 0.5f),
                     (int)(ic_rms + 0.5f));
    telem_queue(n);
}

/* -------------------------------------------------- debug frame ('D') -----
 * Six fields, currents in tenths of an amp and voltages in HUNDREDTHS of a
 * volt, because the question the channel answers is whether vd is 2.5 V or
 * at vmax. vq is also the independent speed cross-check: at id = 0,
 * vq = lambda_F*w_e + Rs*iq. Reading five volatile floats without a critical
 * section is safe: each is a single aligned 32-bit access on Cortex-M4, and
 * they need not be mutually consistent to the tick at 5 Hz against a 20 kHz
 * loop.
 * FIELD COUNT IS FIXED at six; the remote parses on it. iq_ref went to the
 * trace recorder, not here, for exactly that reason.
 *--------------------------------------------------------------------------*/
static void telem_send_dbg(void)
{
    if (!telem_uart_ready()) return;

    int n = snprintf(telem_tx, sizeof(telem_tx),
                     "%c,%d,%d,%d,%d,%d\n",
                     (char)TELEM_DBG_LEAD_CH,
                     telem_scale_i(id_meas,    10.0f),
                     telem_scale_i(iq_meas,    10.0f),
                     telem_scale_i(dbg_vd,    100.0f),
                     telem_scale_i(dbg_vq,    100.0f),
                     telem_scale_i(dbg_vmax,  100.0f));
    telem_queue(n);
}

/* ------------------------------------------------------------- LED --------
 * Above LED_SOLID_PCT: solid. Between THR_STOP_PCT and that: flash rate
 * rises with the dead-man gated throttle. Below THR_STOP_PCT: a short wink
 * once per second for every telemetry frame the UART accepted, so dark and
 * staying dark means transmission has stopped.
 *--------------------------------------------------------------------------*/
static void led_update(uint32_t now)
{
    static uint32_t t_eval = 0, t_hb = 0, hb_seen = 0;
    static uint8_t  ticks = 0, hb_on = 0;

    if (now - t_eval < LED_EVAL_MS) return;
    t_eval = now;

    uint8_t live = ((now - last_valid_rx_tick) < BLE_TIMEOUT_MS) ? 1u : 0u;
    float   thr  = live ? throttle_percent : 0.0f;

    if (thr >= LED_SOLID_PCT) {
        BSP_LED_On(LED_GREEN);
        ticks = 0; hb_on = 0;
        return;
    }

    if (thr < THR_STOP_PCT) {
        ticks = 0;
        if (hb_on) {
            if (--hb_on == 0) BSP_LED_Off(LED_GREEN);
        } else if (now - t_hb >= 1000u) {
            t_hb = now;
            if (dbg_telem_sent != hb_seen) {
                hb_seen = dbg_telem_sent;
                BSP_LED_On(LED_GREEN);
                hb_on = LED_HB_ON_TICKS;
            } else {
                BSP_LED_Off(LED_GREEN);            /* TX has stopped        */
            }
        }
        return;
    }

    hb_on = 0;
    if (ticks) { ticks--; return; }
    BSP_LED_Toggle(LED_GREEN);
    float k = (thr - THR_STOP_PCT) / (LED_SOLID_PCT - THR_STOP_PCT);
    ticks = (uint8_t)(LED_HALF_TICKS_MAX
                      - k * (LED_HALF_TICKS_MAX - LED_HALF_TICKS_MIN) - 0.5f);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* Re-apply everything a regeneration could have reverted, then hold the
     gates hard off before anything switches */
  foc_hw_guard();
  gates_disable();

  /* CORDIC: no .ioc entry, no HAL handle, register access only */
  __HAL_RCC_CORDIC_CLK_ENABLE();

#if TRACE_EN
  /* Trace dump port: no .ioc entry either. A failure here clears
     trace_uart_ok and the recorder goes quiet; the drive is unaffected. */
  trace_uart_init();
#endif

  /* SysTick must never preempt the 20 kHz FOC ISR. HAL_Init sets this from
     TICK_INT_PRIORITY, which defaults to 0 in some CubeMX versions. */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);

  /* DWT cycle counter for ISR duration measurement */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  /* Calibration must happen with the ADCs disabled, i.e. before any
     injected start */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  foc_adc_injected_config();

  ble_rx_arm();

  foc_cal_arm();
  restart_block_until = HAL_GetTick();

  /* 50 percent duties, PWM running so TRGO fires the injected groups */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_MOE_DISABLE(&htim1);                   /* PWM_Start sets MOE    */

  /* Start both injected groups, then enable ONLY the end-of-sequence
     interrupt on ADC1. That single JEOS is the 20 kHz FOC tick and fires
     after all three ADC1 ranks, about 3.3 us after the trigger. ADC2
     finishes its two ranks earlier, so every JDR is valid inside the ISR. */
  HAL_ADCEx_InjectedStart(&hadc2);
  HAL_ADCEx_InjectedStart(&hadc1);
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS);
  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOS);
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

  BSP_LED_Init(LED_GREEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t t_last = 0;
  uint32_t telem_t = 0;
  uint32_t dbg_t = 0;
  uint32_t fault_clear_t = 0;
  uint32_t cal_retry_t = 0;
  uint8_t  temp_seeded = 0;
  uint8_t  ntc_bad_win = 0;
  uint8_t  ntc_good_win = 0;
  uint8_t  vb_bad_win = 0;
  uint8_t  vb_good_win = 0;
  uint8_t  ot_cnt = 0;

  while (1)
  {
    uint32_t now32 = HAL_GetTick();

    led_update(now32);
    ble_rx_watchdog();

#if TRACE_EN
    /* ---- Trace dump --------------------------------------------------
       ONLY with the gates off. It is a blocking transmit of several lines
       per pass, so calling it while modulating would stall the BLE service
       and the dead-man for several milliseconds at a time. The ISR keeps
       running regardless, but the throttle would go stale. */
    {
        foc_state_t ts = foc_state;
        if (ts == FOC_IDLE || ts == FOC_FAULT) trace_service();
    }
#endif

    /* ---- BLE packet assembly, every pass ---- */
    if (ble_rx_ready)
    {
        uint8_t  local[BLE_RX_BUF_LEN];
        uint16_t n;
        __disable_irq();
        n = ble_rx_len;
        if (n > BLE_RX_BUF_LEN) n = BLE_RX_BUF_LEN;
        memcpy(local, ble_rx_copy, n);
        ble_rx_ready = 0;
        __enable_irq();
        parse_ble_bytes(local, n);
    }

    /* ---- Telemetry ----
       Status at 10 Hz while modulating and 4 Hz otherwise, debug at half
       that rate and half a period later so the two never contend. THE
       else-if MATTERS: if a slow pass made both due at once the status frame
       wins, because it carries the state, the fault code and the sentinels
       the dead-man display depends on. */
    {
        foc_state_t s = foc_state;
        uint32_t period = (s == FOC_ALIGN || s == FOC_OPEN_LOOP ||
                           s == FOC_CLOSED_LOOP) ? TELEM_MS_RUN : TELEM_MS_IDLE;

        if (now32 - telem_t >= period) {
            telem_t = now32;
            telem_send();
        } else if ((now32 - dbg_t >= TELEM_DBG_MS_MULT * period) &&
                   (now32 - telem_t >= period / 2u)) {
            dbg_t = now32;
            telem_send_dbg();
        }
    }

    /* ---- 100 ms slow loop ------------------------------------------------
       Everything here comes from accumulators the 20 kHz sequence refreshes.
       There are no HAL ADC calls, so nothing here can stop or reconfigure
       the injected groups. */
    if (now32 - t_last >= 100)
    {
        t_last = now32;

        /* Drain every accumulator in ONE critical section: about thirty
           cycles, well under a microsecond of the 50 us tick. */
        uint32_t nsum, ngood, nbad, vsum, vn, vclip, sn, wn;
        float sa, sb, sc, wsum;
        __disable_irq();
        nsum  = ntc_acc_sum;  ngood = ntc_acc_good; nbad  = ntc_acc_bad;
        vsum  = vb_acc_sum;   vn    = vb_acc_n;     vclip = vb_acc_clip;
        sa    = irms_acc_a;   sb    = irms_acc_b;   sc    = irms_acc_c;
        sn    = irms_acc_n;
        wsum  = w_acc;        wn    = w_acc_n;
        ntc_acc_sum = ntc_acc_good = ntc_acc_bad = 0;
        vb_acc_sum  = vb_acc_n     = vb_acc_clip  = 0;
        irms_acc_a  = irms_acc_b   = irms_acc_c   = 0.0f;
        irms_acc_n  = 0;
        w_acc       = 0.0f;
        w_acc_n     = 0;
        __enable_irq();

        /* ---- NTC (module, not capacitor) ---------------------------------
           Three concerns kept apart: which samples are usable (per sample in
           the ISR), what the good ones mean (mean of up to 2000 conversions,
           about 45x less noise than one snapshot), and whether the sensor is
           faulted (fraction of the window that was bad, separate entry and
           exit counters). */
        uint32_t ntot = ngood + nbad;
        if (ntot > 0u)
        {
            dbg_ntc_bad_pct = (uint8_t)((100u * nbad) / ntot);

            if (ngood > 0u) {
                uint16_t mean = (uint16_t)(nsum / ngood);
                dbg_ntc_raw_mean = mean;
                float t_new = ntc_temp_c(mean);
                if (!temp_seeded) {
                    module_temp_c = t_new;
                    temp_seeded = 1;
                } else {
                    module_temp_c += NTC_IIR_ALPHA * (t_new - module_temp_c);
                }
            }
            /* ngood == 0 holds the last good temperature: never invent a
               number for a channel that produced nothing usable. */

            uint8_t win_bad =
                (nbad * NTC_BAD_FRAC_DEN > ntot * NTC_BAD_FRAC_NUM) ? 1u : 0u;

            if (!ntc_fault) {
                if (win_bad) {
                    if (++ntc_bad_win >= NTC_BAD_WIN) { ntc_fault = 1; ntc_good_win = 0; }
                } else {
                    ntc_bad_win = 0;
                }
            } else {
                if (!win_bad) {
                    if (++ntc_good_win >= NTC_GOOD_WIN) { ntc_fault = 0; ntc_bad_win = 0; }
                } else {
                    ntc_good_win = 0;
                }
            }
        }
        /* ntot == 0 means the ISR did not run at all during the window:
           leave every temperature state alone, that is a timer or ADC
           failure, not a thermistor failure. */

        /* ---- Bus voltage -------------------------------------------------
           THE FAULT DECISION ONLY. The modulator and the observer run on the
           1 ms vbus_fast estimate in the ISR, because a supply entering
           constant current collapses the bus far faster than this window can
           follow. Watch dbg_vbus_v_raw during a constant-current event: if
           the supply folds below the floor for three windows the sense fault
           sets, the interlock returns to IDLE, 300 ms later it clears and
           the drive restarts, giving a roughly 1 Hz restart oscillator that
           looks like a brownout. A return to state 0 means the MCU rebooted;
           a return to state 1 means only the interlock cycled.
           A speed ceiling also comes from here: vmax = 0.43*vbus, so a
           sagging supply caps the reachable RPM before any current limit
           does. */
        if (vn > 0u)
        {
            uint16_t vmean = (uint16_t)(vsum / vn);
            dbg_vbus_raw_mean = vmean;
            dbg_vbus_clip_pct = (uint8_t)((100u * vclip) / vn);

            float v = (float)vmean * (VREF / ADC_FS) * VBUS_GAIN;
            dbg_vbus_v_raw = v;

            uint8_t win_bad = (v < VBUS_PLAUS_MIN) ? 1u : 0u;
            if (!vbus_sense_fault) {
                if (win_bad) {
                    if (++vb_bad_win >= VBUS_BAD_WIN) { vbus_sense_fault = 1; vb_good_win = 0; }
                } else {
                    vb_bad_win = 0;
                }
            } else {
                if (!win_bad) {
                    if (++vb_good_win >= VBUS_GOOD_WIN) { vbus_sense_fault = 0; vb_bad_win = 0; }
                } else {
                    vb_good_win = 0;
                }
            }

            vbus_v = vbus_sense_fault ? VBUS_FALLBACK
                                      : CLAMPF(v, VBUS_PLAUS_MIN, VBUS_PLAUS_MAX);
        }

        /* ---- Phase current RMS over the window ---------------------------
           sn == 0 means the ISR did not run, or the drive spent the whole
           window in FOC_OFFSET_CAL. Hold the last values rather than
           reporting a false zero: a genuine zero is what an idle but running
           drive produces and the two must not look alike, and holding also
           keeps charging the capacitor thermal budget. */
        if (sn > 0u) {
            float inv = 1.0f / (float)sn;
            ia_rms = sqrtf(sa * inv);
            ib_rms = sqrtf(sb * inv);
            ic_rms = sqrtf(sc * inv);
        }

        cap_limit_update();

        /* ---- Mechanical RPM ----------------------------------------------
           Window MEAN of the electrical speed, over the same 2000 ticks the
           RMS currents use. The accumulator already selected the right
           source per tick: the commanded ramp in open loop, the tracking
           observer in closed loop, zero elsewhere.
           wn == 0 means the ISR did not run during the window, so hold the
           last value rather than reporting a false zero.
           Because the tracker coasts, closed loop with the EMF lost reports
           the coasted speed for up to SYNC_LOST_N ticks (100 ms) before the
           shutdown fires; read dbg_emf_bad_max if the displayed RPM looks
           like it is holding when the machine is not. */
        if (wn > 0u) {
            float w_e = wsum / (float)wn;
            motor_rpm = (w_e / POLE_PAIRS) * (60.0f / (2.0f * (float)M_PI));
        }

        foc_state_t s = foc_state;

        /* ---- Re-arm a FULL calibration if the HASS never came up ---------
           The HASS runs off a rail derived from the battery and is not alive
           when the Nucleo is on USB before the pack is connected, so the
           first calibration would otherwise block the start interlock
           forever. Suppressed in lab mode, where the start is not gated on
           idc_sense_ok and this would re-enter FOC_OFFSET_CAL once a second
           for as long as the board was powered. */
        if (!LAB_TEST_MODE && s == FOC_IDLE && !idc_sense_ok &&
            !vbus_sense_fault && (now32 - cal_retry_t) >= CAL_RETRY_MS) {
            cal_retry_t = now32;
            dbg_cal_retry++;
            foc_cal_arm();
        }

        /* ---- Overtemperature trip, debounced over OT_CONFIRM_N passes ----
           The module thermal constant is orders of magnitude longer than
           300 ms, so the debounce costs nothing in protection. MODULE ONLY:
           there is no capacitor overtemperature trip because there is no
           capacitor sensor. */
        if (!ntc_fault && temp_seeded && module_temp_c > TEMP_TRIP_C) {
            if (ot_cnt < OT_CONFIRM_N) ot_cnt++;
        } else if (ot_cnt) {
            ot_cnt--;
        }
        if (ot_cnt >= OT_CONFIRM_N && s != FOC_FAULT && s != FOC_OFFSET_CAL) {
            if (!LAB_TEST_MODE) foc_trip(FAULT_OVERTEMP);
        }

        /* ---- Fault latch clear: throttle at zero for 2 s -----------------
           Uses the DEAD-MAN GATED throttle: reading throttle_percent
           directly freezes it at its last value when the link drops, so a
           fault raised at 40 percent could never be cleared without a power
           cycle. Clearing a fault does NOT clear the capacitor thermal
           estimate, because a trip is not a cooling event. The clear takes
           FAIL_HOLD_MS on top, because a latched fault is the one case where
           the feedback is most likely to have been the thing that was wrong.
           Note that with fixed phase offsets there is no re-check of the ACS
           on this path: a FAULT_ACS_SAT raised at boot clears to IDLE after
           2 s. Replace the assignment with foc_cal_arm() if that matters.
           The trace buffer survives this: it is only overwritten by the next
           foc_reset(), so a fault capture can still be dumped after the
           latch clears. */
        uint8_t live = ((now32 - last_valid_rx_tick) < BLE_TIMEOUT_MS) ? 1u : 0u;
        float thr_gated = live ? throttle_percent : 0.0f;

        if (foc_state == FOC_FAULT) {
            if (thr_gated < THR_STOP_PCT) {
                if (fault_clear_t == 0) fault_clear_t = now32;
                if (now32 - fault_clear_t > 2000) {
                    fault_code    = FAULT_NONE;
                    fault_clear_t = 0;
                    ot_cnt        = 0;
                    restart_block_until = now32 + FAIL_HOLD_MS;
                    foc_state     = FOC_IDLE;
                }
            } else {
                fault_clear_t = 0;
            }
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  *
  * Injected ranks are configured entirely in foc_adc_injected_config(), and
  * NO regular channel is configured because no regular conversion is ever
  * started. NbrOfConversion stays at 1 and SQR1 keeps its reset value; both
  * are inert while ADSTART stays clear.
  */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;   /* 42.5 MHz       */
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* CCR is shared by the ADC1/ADC2 pair, so this covers both */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  */
static void MX_ADC2_Init(void)
{
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  */
static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = PWM_ARR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /* tDTS = 5.882 ns   */
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_ARR / 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* OSSR/OSSI disabled: with MOE clear the TIM1 pins go Hi-Z, which is what
     keeps the drivers safe between reset and MX_GPIO_Init. Break is disabled
     because there is NO hardware BRK fault path on this board; overcurrent
     protection is entirely in software, which is why lab test mode must not
     be extended to the phase overcurrent trip. The interim report describes
     a BKIN comparator trip that was never built and needs correcting. */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = DEADTIME_DTG;   /* 0xCA = 202, 1.98 us    */
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief USART1 Initialization Function
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;                   /* HM-10 factory default  */
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Both below the ADC ISR at priority 1 */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GATE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GATE_EN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* PC10 and the analog pins are re-applied in foc_hw_guard();
   PA2/PA3 are configured in trace_uart_init() */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* 20 kHz FOC tick: ADC1 injected end of sequence. ADC2 shares the same TIM1
   TRGO trigger and finishes its shorter sequence first, so JDR1 on both ADCs
   holds simultaneous i_a / i_b samples taken at the trigger instant. */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        foc_isr();
    }
}

/* Copy the packet out of the live DMA buffer and re-arm immediately, so a
   new packet landing during the main-loop parse cannot corrupt the bytes
   being read. If the arm fails because the handle was locked, the main-loop
   watchdog picks it up on the next pass. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        uint16_t n = (Size > BLE_RX_BUF_LEN) ? BLE_RX_BUF_LEN : Size;
        memcpy(ble_rx_copy, ble_rx_buf, n);
        ble_rx_len   = n;
        ble_rx_ready = 1;
        ble_rx_arm();
    }
}

/* Re-arm after overrun or framing errors caused by BLE reconnect churn.
   ONLY the receive side is aborted and restarted: aborting the whole UART
   would kill an in-flight telemetry transfer and leave gState stuck.
   USART2 is polled transmit only and is not handled here. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_AbortReceive(huart);
        ble_rx_arm();
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);  /* gates off */
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
