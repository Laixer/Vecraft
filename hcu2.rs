let mut lock_led = gpioc.pc12.into_push_pull_output();
lock_led.set_low();

// let power_swtich1 = gpiod.pd11.into_push_pull_output();
let power_swtich1 = gpiod.pd10.into_push_pull_output();
let power_swtich2 = gpioa.pa12.into_push_pull_output();
let power_swtich3 = gpioc.pc1.into_push_pull_output();


// Verplaats TIM3 -> TIM8:
let (_, (pwm_high4, pwm_low4, pwm_high7, pwm_low7)) = ctx
.device
.TIM8
.pwm_advanced(
    (
        gpioc.pc6.into_alternate(),
        gpioc.pc7.into_alternate(),
        gpioc.pc8.into_alternate(),
        gpioc.pc9.into_alternate(),
    ),
    ccdr.peripheral.TIM8, // 8?
    &ccdr.clocks,
)
.frequency(100.Hz())
.period(32_767)
.finalize();

// In the new HCU 5 => 6, and 6 => 5
let (_, (pwm_high5, pwm_low5, pwm_high6, pwm_low6)) = ctx
.device
.TIM1
.pwm_advanced(
(
        gpioa.pa10.into_alternate(),
        gpioa.pa11.into_alternate(),
        gpioa.pa8.into_alternate(),
        gpioa.pa9.into_alternate(),
),
ccdr.peripheral.TIM1,
&ccdr.clocks,
)
.frequency(100.Hz())
.period(32_767)
.finalize();

// Add this timer
let (_, (pwm_high8, pwm_low8, pwm_high9, pwm_low9)) = ctx
.device
.TIM3
.pwm_advanced(
    (
        gpiob.pb0.into_alternate(),
        gpiob.pb1.into_alternate(),
        gpioa.pa6.into_alternate(),
        gpioa.pa7.into_alternate(),
    ),
    ccdr.peripheral.TIM3,
    &ccdr.clocks,
)
.frequency(100.Hz())
.period(32_767)
.finalize();

// Add these gates
let gate8 = vecraft::lsgc::Gate::new(pwm_high8, pwm_low8);
let gate9 = vecraft::lsgc::Gate::new(pwm_high9, pwm_low9);

// Change the gate_lock
let mut gate_lock = vecraft::lsgc::GateLock {
lockout0: power_swtich1,
lockout1: power_swtich2,
lockout2: power_swtich3,
};

// Add the gates to the lock
gate8, // IN2
gate9, // IN1
