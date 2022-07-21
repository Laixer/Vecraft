pub enum GateSide<T1, T2> {
    Up(T1),
    Down(T2),
    Hold,
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
            GateSide::Hold => {
                self.channel_down.disable();
                self.channel_up.disable();
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

    fn disable(&mut self) {
        self.channel_down.disable();
        self.channel_up.disable();
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

pub struct GateControl {
    pub lockout0: PinLockout0,
    pub lockout1: PinLockout1,
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
    fn disable(&mut self) {
        self.gate0.disable();
        self.gate1.disable();
        self.gate2.disable();
        self.gate3.disable();
        self.gate4.disable();
        self.gate5.disable();
        self.gate6.disable();
        self.gate7.disable();
    }

    pub fn lock(&mut self) {
        self.disable();
        self.lockout0.set_low();
        self.lockout1.set_low();
    }

    pub fn unlock(&mut self) {
        self.disable();
        self.lockout0.set_high();
        self.lockout1.set_high();
    }
}
