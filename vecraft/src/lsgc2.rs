use stm32h7xx_hal::{
    device::{TIM1, TIM2, TIM3, TIM4, TIM8},
    pwm::{ComplementaryDisabled, ComplementaryImpossible, Pwm},
};

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

pub type Gate0 = Gate<Pwm<TIM4, 0, ComplementaryImpossible>, Pwm<TIM4, 1, ComplementaryImpossible>>;
pub type Gate1 = Gate<Pwm<TIM2, 1, ComplementaryImpossible>, Pwm<TIM2, 0, ComplementaryImpossible>>;
pub type Gate2 = Gate<Pwm<TIM2, 3, ComplementaryImpossible>, Pwm<TIM2, 2, ComplementaryImpossible>>;
pub type Gate3 = Gate<Pwm<TIM4, 2, ComplementaryImpossible>, Pwm<TIM4, 3, ComplementaryImpossible>>;
pub type Gate4 = Gate<Pwm<TIM8, 0, ComplementaryDisabled>, Pwm<TIM8, 1, ComplementaryDisabled>>;
pub type Gate5 = Gate<Pwm<TIM1, 2, ComplementaryDisabled>, Pwm<TIM1, 3, ComplementaryImpossible>>;
pub type Gate6 = Gate<Pwm<TIM1, 0, ComplementaryDisabled>, Pwm<TIM1, 1, ComplementaryDisabled>>;
pub type Gate7 = Gate<Pwm<TIM8, 2, ComplementaryDisabled>, Pwm<TIM8, 3, ComplementaryImpossible>>;
pub type Gate8 = Gate<Pwm<TIM3, 2, ComplementaryImpossible>, Pwm<TIM3, 3, ComplementaryImpossible>>;
pub type Gate9 = Gate<Pwm<TIM3, 0, ComplementaryImpossible>, Pwm<TIM3, 1, ComplementaryImpossible>>;

// pub type PinLockout0 =
//     stm32h7xx_hal::gpio::PD11<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>;
pub type PinLockout0 =
    stm32h7xx_hal::gpio::PD10<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>;
pub type PinLockout1 =
    stm32h7xx_hal::gpio::PA12<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>;
pub type PinLockout2 =
    stm32h7xx_hal::gpio::PC1<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>;

pub struct GateLock {
    pub lockout0: PinLockout0,
    pub lockout1: PinLockout1,
    pub lockout2: PinLockout2,
}

impl GateLock {
    pub fn lock(&mut self) {
        self.lockout0.set_low();
        self.lockout1.set_low();
        self.lockout2.set_low();
    }

    pub fn unlock(&mut self) {
        self.lockout0.set_high();
        self.lockout1.set_high();
        self.lockout2.set_high();
    }

    pub fn is_locked(&self) -> bool {
        self.lockout0.is_set_low() && self.lockout1.is_set_low() && self.lockout2.is_set_low()
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
    pub gate8: Gate8,
    pub gate9: Gate9,
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
        self.gate8.set_value(GateSide::Hold(0, 0));
        self.gate9.set_value(GateSide::Hold(0, 0));
    }
}
