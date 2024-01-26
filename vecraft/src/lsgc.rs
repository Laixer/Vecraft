pub enum GateSide<T1, T2> {
    Up(T1),
    Down(T2),
    Hold(T1, T2),
}

pub struct Gate<T1, T2> {
    channel_up: T1,
    channel_down: T2,
}

impl<T1, T2> Gate<T1, T2>
where
    T1: stm32h7xx_hal::hal::PwmPin,
    T2: stm32h7xx_hal::hal::PwmPin,
{
    pub fn new(channel_up: T1, channel_down: T2) -> Self {
        Self {
            channel_up,
            channel_down,
        }
    }

    pub fn set_value(&mut self, value: GateSide<T1::Duty, T2::Duty>) -> &mut Self {
        match value {
            GateSide::Hold(value1, value2) => {
                self.channel_up.set_duty(value1);
                self.channel_down.set_duty(value2);
                self.channel_up.disable();
                self.channel_down.disable();
            }
            GateSide::Up(value) => {
                self.channel_down.disable();
                self.channel_up.set_duty(value);
                self.channel_up.enable();
            }
            GateSide::Down(value) => {
                self.channel_up.disable();
                self.channel_down.set_duty(value);
                self.channel_down.enable();
            }
        }
        self
    }
}

use stm32h7xx_hal::{
    device::{TIM1, TIM2, TIM3, TIM4},
    pwm::{ActiveHigh, ComplementaryDisabled, ComplementaryImpossible, Pwm, C1, C2, C3, C4},
};

pub type Gate0 = Gate<
    Pwm<TIM4, C1, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
    Pwm<TIM4, C2, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
>;
pub type Gate1 = Gate<
    Pwm<TIM2, C2, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
    Pwm<TIM2, C1, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
>;
pub type Gate2 = Gate<
    Pwm<TIM2, C4, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
    Pwm<TIM2, C3, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
>;
pub type Gate3 = Gate<
    Pwm<TIM4, C3, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
    Pwm<TIM4, C4, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
>;
pub type Gate4 = Gate<
    Pwm<TIM3, C1, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
    Pwm<TIM3, C2, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
>;
pub type Gate5 = Gate<
    Pwm<TIM1, C3, ComplementaryDisabled, ActiveHigh, ActiveHigh>,
    Pwm<TIM1, C4, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
>;
pub type Gate6 = Gate<
    Pwm<TIM1, C1, ComplementaryDisabled, ActiveHigh, ActiveHigh>,
    Pwm<TIM1, C2, ComplementaryDisabled, ActiveHigh, ActiveHigh>,
>;
pub type Gate7 = Gate<
    Pwm<TIM3, C3, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
    Pwm<TIM3, C4, ComplementaryImpossible, ActiveHigh, ActiveHigh>,
>;

pub type PinLockout0 =
    stm32h7xx_hal::gpio::PD11<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>;
pub type PinLockout1 =
    stm32h7xx_hal::gpio::PA12<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>;

pub struct GateLock {
    pub lockout0: PinLockout0,
    pub lockout1: PinLockout1,
}

impl GateLock {
    pub fn lock(&mut self) {
        self.lockout0.set_low();
        self.lockout1.set_low();
    }

    pub fn unlock(&mut self) {
        self.lockout0.set_high();
        self.lockout1.set_high();
    }

    pub fn is_locked(&self) -> bool {
        self.lockout0.is_set_low() && self.lockout1.is_set_low()
    }
}

pub struct GateControl {
    pub gate0: Gate0,
    pub gate1: Gate1,
    pub gate2: Gate2,
    pub gate3: Gate3,
    pub gate4: Gate4,
    pub gate5: Gate5,
    pub gate6: Gate6,
    pub gate7: Gate7,
}

impl GateControl {
    pub fn reset(&mut self) {
        self.gate0.set_value(GateSide::Hold(0, 0));
        self.gate1.set_value(GateSide::Hold(0, 0));
        self.gate2.set_value(GateSide::Hold(0, 0));
        self.gate3.set_value(GateSide::Hold(0, 0));
        self.gate4.set_value(GateSide::Hold(0, 0));
        self.gate5.set_value(GateSide::Hold(0, 0));
        self.gate6.set_value(GateSide::Hold(0, 0));
        self.gate7.set_value(GateSide::Hold(0, 0));
    }
}
