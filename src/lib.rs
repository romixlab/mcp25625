#![no_std]

use core::fmt::{Formatter};
use core::marker;
use core::fmt;
use embedded_hal::blocking::spi::{Write, Transfer};
use embedded_hal::digital::v2::OutputPin;
use cortex_m::asm::delay;

/// Shows that register has `read` method.
pub trait Readable {}

/// Shows that register has `write` method.
/// Registers marked with `Readable` has also `modify` method.
pub trait Writable {}

/// Reset value of the register used in `write` and `reset` methods.
pub trait ResetValue {
    /// Register underlying type
    type Type;
    fn reset_value() -> Self::Type;
}

/// Address of the register used in all `read`, `write`, ... methods.
pub trait Address {
    type Type;
    fn address() -> Self::Type;
}

/// Register field reader.
pub struct R<U, M> {
    bits: U,
    _marker: marker::PhantomData<M>
}

impl<U, M> R<U, M>
    where
        U: Copy
{
    /// Create new instance of reader
    pub fn new(bits: U) -> Self {
        Self {
            bits,
            _marker: marker::PhantomData
        }
    }
    /// Read raw bits from register/bitfield
    pub fn bits(&self) -> U {
        self.bits
    }
}

impl <U, T, FI> PartialEq<FI> for R<U, T>
    where
        U: PartialEq,
        FI: Copy + Into<U>
{
    // Needed in register implementations
    fn eq(&self, other: &FI) -> bool {
        self.bits.eq(&(*other).into())
    }
}

impl<FI> R<bool, FI> {
    /// Returns value of the bit.
    pub fn bit(&self) -> bool { self.bits }
    /// Returns `true` if the bit is clear (0).
    pub fn bit_is_clear(&self) -> bool { !self.bit() }
    /// Returns `true` is the bit is set (1).
    pub fn bit_is_set(&self) -> bool { self.bit() }
}

/// Register writer
pub struct W<U, M> {
    bits: U,
    _marker: marker::PhantomData<M>
}

impl<U, M> W<U, M> {
    /// Writes raw bits to the register.
    pub unsafe fn bits(&mut self, bits: U) -> &mut Self {
        self.bits = bits;
        self
    }
}

//pub struct _BFPCTRL;
//type BFPCTRL = GenericReg<u8, u8, _BFPCTRL>;

impl Readable for BFPCTRL {}
impl Writable for BFPCTRL {}

pub struct BFPCTRL;
impl BFPCTRL {
    const ADDR: u8 = 0b0000_1100;
}
type BfpctrlR = R<u8, BFPCTRL>;
type BfpctrlW = W<u8, BFPCTRL>;

impl ResetValue for BFPCTRL {
    type Type = u8;
    fn reset_value() -> Self::Type { 0 }
}

impl Address for BFPCTRL {
    type Type = u8;
    fn address() -> Self::Type { Self::ADDR }
}

pub struct CANSTAT;
impl Address for CANSTAT {
    type Type = u8;
    fn address() -> Self::Type { 0b0000_1110 }
}

pub struct CANCTRL;
impl Address for CANCTRL {
    type Type = u8;
    fn address() -> Self::Type { 0b0000_1111 }
}

pub struct CNF1;
impl Address for CNF1 {
    type Type = u8;
    fn address() -> Self::Type { 0b0010_1010 }
}

pub struct CNF2;
impl Address for CNF2 {
    type Type = u8;
    fn address() -> Self::Type { 0b0010_1001 }
}


pub struct CNF3;
impl Address for CNF3 {
    type Type = u8;
    fn address() -> Self::Type { 0b0010_1000 }
}


#[derive(Clone, Copy, Debug, PartialEq)]
pub enum B0fsA {
    High,
    Low
}

impl From<B0fsA> for bool {
    fn from(variant: B0fsA) -> Self {
        match variant {
            B0fsA::High => true,
            B0fsA::Low => false
        }
    }
}

type B0fsR = R<bool, B0fsA>;
impl B0fsR {
    pub fn variant(&self) -> B0fsA {
        match self.bits {
            true => B0fsA::High,
            false => B0fsA::Low
        }
    }
    pub fn is_high(&self) -> bool {
        *self == B0fsA::High
    }
    pub fn is_low(&self) -> bool {
        *self == B0fsA::Low
    }
}

pub struct B0fsW<'a> {
    w: &'a mut BfpctrlW
}

impl<'a> B0fsW<'a> {
    pub fn set_bit(self) -> &'a mut BfpctrlW {
        self.bit(true)
    }
    pub fn clear_bit(self) -> &'a mut BfpctrlW {
        self.bit(false)
    }
    pub fn bit(self, value: bool) -> &'a mut BfpctrlW {
        self.w.bits = (self.w.bits & !(0b1 << 5)) | (((value as u8) & 0b1) << 5);
        self.w
    }
    pub fn high(self) -> &'a mut BfpctrlW {
        self.bit(true)
    }
    pub fn low(self) -> &'a mut BfpctrlW {
        self.bit(false)
    }
}

///
///
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum B0feA {
    Enabled,
    HighImpedance
}

impl From<B0feA> for bool {
    fn from(variant: B0feA) -> Self {
        match variant {
            B0feA::Enabled => true,
            B0feA::HighImpedance => false
        }
    }
}

type B0feR = R<bool, B0feA>;
impl B0feR {
    pub fn variant(&self) -> B0feA {
        match self.bits {
            true => B0feA::Enabled,
            false => B0feA::HighImpedance
        }
    }
    pub fn is_high(&self) -> bool {
        *self == B0feA::Enabled
    }
    pub fn is_low(&self) -> bool {
        *self == B0feA::HighImpedance
    }
}

pub struct B0feW<'a> {
    w: &'a mut BfpctrlW
}

impl<'a> B0feW<'a> {
    pub fn set_bit(self) -> &'a mut BfpctrlW {
        self.bit(true)
    }
    pub fn clear_bit(self) -> &'a mut BfpctrlW {
        self.bit(false)
    }
    pub fn bit(self, value: bool) -> &'a mut BfpctrlW {
        self.w.bits = (self.w.bits & !(0b1 << 3)) | (((value as u8) & 0b1) << 3);
        self.w
    }
    pub fn enable(self) -> &'a mut BfpctrlW {
        self.bit(true)
    }
}
///
///
///
///
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum B0fmA {
    Interrupt,
    Output
}

impl From<B0fmA> for bool {
    fn from(variant: B0fmA) -> Self {
        match variant {
            B0fmA::Interrupt => true,
            B0fmA::Output => false
        }
    }
}

type B0fmR = R<bool, B0fmA>;
impl B0fmR {
    pub fn variant(&self) -> B0fmA {
        match self.bits {
            true => B0fmA::Interrupt,
            false => B0fmA::Output
        }
    }
    pub fn is_high(&self) -> bool {
        *self == B0fmA::Interrupt
    }
    pub fn is_low(&self) -> bool {
        *self == B0fmA::Output
    }
}

pub struct B0fmW<'a> {
    w: &'a mut BfpctrlW
}

impl<'a> B0fmW<'a> {
    pub fn set_bit(self) -> &'a mut BfpctrlW {
        self.bit(true)
    }
    pub fn clear_bit(self) -> &'a mut BfpctrlW {
        self.bit(false)
    }
    pub fn bit(self, value: bool) -> &'a mut BfpctrlW {
        self.w.bits = (self.w.bits & !(0b1 << 0)) | (((value as u8) & 0b1) << 0);
        self.w
    }
    pub fn output(self) -> &'a mut BfpctrlW {
        self.bit(false)
    }
    pub fn interrupt(self) -> &'a mut BfpctrlW {
        self.bit(true)
    }
}
///
///

impl BfpctrlR {
    pub fn b0fs(&self) -> B0fsR { B0fsR::new(((self.bits >> 5) & 0x01) != 0) }
    pub fn b0fe(&self) -> B0feR { B0feR::new(((self.bits >> 3) & 0x01) != 0) }
    pub fn b0fm(&self) -> B0feR { B0feR::new(((self.bits >> 0) & 0x01) != 0) }
}

impl BfpctrlW {
    pub fn b0fs(&mut self) -> B0fsW {
        B0fsW {
            w: self
        }
    }

    pub fn b0fe(&mut self) -> B0feW {
        B0feW {
            w: self
        }
    }

    pub fn b0fm(&mut self) -> B0fmW {
        B0fmW {
            w: self
        }
    }
}

#[derive(Debug)]
pub enum Error<E> {
    /// Late collision
    LateCollision,
    /// SPI error
    Spi(E),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Spi(e)
    }
}

pub struct Mcp25625Ral<SPI, CS> {
    spi: SPI,
    cs: CS,
    one_cp: u32 // number of cpu cycles per one sck period
}

pub enum McpFastRxRead {
    RXB0SIDH = 0b00,
    RXB0D0 = 0b01,
    RXB1SIDH = 0b10,
    RXB1D0 = 0b11
}

impl<E, SPI, CS> Mcp25625Ral<SPI, CS>
    where
        SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
        CS: OutputPin
{
    const RESET_CMD:       u8 = 0b1100_0000;
    const READ_CMD:        u8 = 0b0000_0011;
    const READ_RX_BUF_CMD: u8 = 0b1001_0000; // 1001_0nm0
    const WRITE_CMD:       u8 = 0b0000_0010;
    //const LOAD_TX_CMD:     u8 = 0b0100_0000; // 0100_0abc
    const RTS_CMD:         u8 = 0b1000_0000; // 1000_0nnn
    //const READ_STATUS_CMD: u8 = 0b1010_0000;
    //const RX_STATUS:       u8 = 0b1011_0000;
    const BIT_MODIFY_CMD:  u8 = 0b0000_0101;

    pub fn new(spi: SPI, cs: CS, one_cp: u32) -> Self {
        Mcp25625Ral {
            spi,
            cs,
            one_cp
        }
    }

    pub fn write_reg(&mut self, addr: u8, byte: u8) {
        self.cs.set_low().ok();
        let _ = self.spi.write(&[Self::WRITE_CMD, addr, byte]);
        delay(self.one_cp);
        self.cs.set_high().ok();
        delay(self.one_cp * 3);
    }

    pub fn bit_modify(&mut self, addr: u8, mask: u8, bits: u8) {
        self.cs.set_low().ok();
        let _ = self.spi.write(&[Self::BIT_MODIFY_CMD, addr, mask, bits]);
        delay(self.one_cp);
        self.cs.set_high().ok();
        delay(self.one_cp * 3);
    }

    fn write_many2(&mut self, addr: u8, d0: &[u8], d1: &[u8]) {
        self.cs.set_low().ok();
        let _ = self.spi.write(&[Self::WRITE_CMD, addr]);
        let _ = self.spi.write(d0);
        let _ = self.spi.write(d1);
        delay(self.one_cp);
        self.cs.set_high().ok();
        delay(self.one_cp * 3);
    }

    pub fn write_raw(&mut self, bytes: &[u8]) {
        self.cs.set_low().ok();
        let _ = self.spi.write(bytes);
        delay(self.one_cp);
        self.cs.set_high().ok();
        delay(self.one_cp * 3);
    }

    pub fn read_reg(&mut self, addr: u8) -> u8 {
        self.cs.set_low().ok();
        let _ = self.spi.write(&[Self::READ_CMD, addr]);
        let mut buf = [0u8; 1];
        let r = self.spi.transfer(&mut buf);
        delay(self.one_cp);
        self.cs.set_high().ok();
        delay(self.one_cp * 3);
        r.unwrap_or(&[0u8])[0]
    }

    // Reads from 0x61/0x66 or 0x71/0x76 for buffers 0/1
    pub fn read_rxbuf(&mut self, addr: McpFastRxRead, buf: &mut [u8]) {
        self.cs.set_low().ok();
        let _ = self.spi.write(&[Self::READ_RX_BUF_CMD | ((addr as u8) << 1)]);
        let _ = self.spi.transfer(buf);
        delay(self.one_cp);
        self.cs.set_high().ok();
        delay(self.one_cp * 3);
    }

    pub fn read<REG>(&mut self) -> R<u8, REG>
        where
            REG: Address<Type=u8>
    {
        let bits = self.read_reg(REG::address());
        R { bits, _marker: marker::PhantomData }
    }

    pub fn write<REG, F>(&mut self, f: F)
        where
            F: FnOnce(&mut W<u8, REG>) -> &mut W<u8, REG>,
            REG: ResetValue<Type=u8> + Address<Type=u8>
    {
        let bits = f(&mut W{ bits: REG::reset_value(), _marker: marker::PhantomData }).bits;
        self.write_reg(REG::address(), bits);
    }
    //
    // pub fn bfpctrl_reset(&mut self) {
    //
    // }
    //
    // pub fn bfpctrl_write_with_zero() {
    //
    // }
    //
    // pub fn bfpctrl_modify<F>(&mut self, f: F)
    //     where
    //             for<'w> F: FnOnce(&R<u8, BFPCTRL>, &'w mut W<u8, BFPCTRL>) -> &'w mut W<u8, BFPCTRL>
    // {
    //     let mem_r = self.mem;
    //     self.mem = f(&R{bits: mem_r, _marker: marker::PhantomData},
    //                  &mut W{bits: mem_r, _marker: marker::PhantomData}).bits;
    //     //self.bfpctrl.modify(|_, byte| { *mem_b = byte; }, |_| mem_r, f);
    // }

    //pub fn bfpctrl(|reg| reg.)
}

pub struct MCP25625<SPI, CS> {
    ral: Mcp25625Ral<SPI, CS>,
}

pub struct MCP25625Config {
    pub brp: u8, // 0-63, Tq = 2*(brp+1) / Fosc
    // Tq/bit = 5-25
    // Nbt = Tq*(sync_seg=1 + prop_seg + ph_seg1 + ph_seg2)
    // Nbr = 1/Nbt
    pub prop_seg: u8, // 1-8 Tq
    pub ph_seg1: u8, // 1-8 Tq
    pub ph_seg2: u8, // 2-8 Tq
    pub sync_jump_width: u8, // 1-4 Tq
}

#[derive(Debug)]
pub enum McpErrorKind {
    WrongTqSum,
    WrongBrp,
    WrongSJW,
    WrongPropSeg,
    WrongPhaseSeg1,
    WrongPhaseSeg2,
    ConfigRequestFailed,
    RegVerifyError(u8),
    NoTxSlotsAvailable,
    TooBig
}

/// Convert enum to bits
trait Bits {
    fn bits(self: &Self) -> u8;
}

#[derive(PartialEq)]
pub enum McpOperationMode {
    Normal,
    Sleep,
    Loopback,
    ListenOnly,
    Configuration,
    Wrong
}


impl Bits for McpOperationMode {
    fn bits(&self) -> u8 {
        match self {
            McpOperationMode::Normal => 0b000,
            McpOperationMode::Sleep => 0b001,
            McpOperationMode::Loopback => 0b010,
            McpOperationMode::ListenOnly => 0b011,
            McpOperationMode::Configuration => 0b100,
            _ => 0b000
        }
    }
}

pub enum McpStatInterruptFlags {
    NoInterrupt,
    ErrorInterrupt,
    WakeUpInterrupt,
    TXB0Interrupt,
    TXB1Interrupt,
    TXB2Interrupt,
    RXB0Interrupt,
    RXB1Interrupt
}

pub struct McpErrorFlags {
    pub bits: u8,
}

impl McpErrorFlags {
    pub fn rx1ovr_is_set(&self) -> bool { self.bits & (1u8 << 7) != 0 }
    pub fn rx0ovr_is_set(&self) -> bool { self.bits & (1u8 << 6) != 0 }
    pub fn txbo_is_set(&self) -> bool { self.bits & (1u8 << 5) != 0 }
    pub fn txep_is_set(&self) -> bool { self.bits & (1u8 << 4) != 0 }
    pub fn rxep_is_set(&self) -> bool { self.bits & (1u8 << 3) != 0 }
    pub fn txwar_is_set(&self) -> bool { self.bits & (1u8 << 2) != 0 }
    pub fn rxwar_is_set(&self) -> bool { self.bits & (1u8 << 1) != 0 }
    pub fn ewarn_is_set(&self) -> bool { self.bits & (1u8 << 0) != 0 }
    pub fn is_err(&self) -> bool { self.bits != 0 }
    pub fn is_ok(&self) -> bool { self.bits == 0 }
}

impl fmt::Debug for McpErrorFlags {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let _ = write!(f, "EFLG=(");
        if self.rx1ovr_is_set() {
            let _ = write!(f, "RX1OVR, ");
        };
        if self.rx0ovr_is_set() {
            let _ = write!(f, "RX0OVR, ");
        };
        if self.txbo_is_set() {
            let _ = write!(f, "BUSOFF, ");
        };
        if self.txep_is_set() {
            let _ = write!(f, "TXEP, ");
        };
        if self.rxep_is_set() {
            let _ = write!(f, "RXEP, ");
        };
        if self.txwar_is_set() {
            let _ = write!(f, "TXWAR, ");
        };
        if self.rxwar_is_set() {
            let _ = write!(f, "RXWAR, ");
        };
        if self.ewarn_is_set() {
            let _ = write!(f, "EWARN, ");
        };
        write!(f, ")")
    }
}

pub struct McpInterruptFlags {
    pub bits: u8,
}

impl McpInterruptFlags {
    pub fn merrf_is_set(&self) -> bool { self.bits & (1u8 << 7) != 0 }
    pub fn wakif_is_set(&self) -> bool { self.bits & (1u8 << 6) != 0 }
    pub fn errif_is_set(&self) -> bool { self.bits & (1u8 << 5) != 0 }
    pub fn tx2if_is_set(&self) -> bool { self.bits & (1u8 << 4) != 0 }
    pub fn tx1if_is_set(&self) -> bool { self.bits & (1u8 << 3) != 0 }
    pub fn tx0if_is_set(&self) -> bool { self.bits & (1u8 << 2) != 0 }
    pub fn rx1if_is_set(&self) -> bool { self.bits & (1u8 << 1) != 0 }
    pub fn rx0if_is_set(&self) -> bool { self.bits & (1u8 << 0) != 0 }
}

impl fmt::Debug for McpInterruptFlags {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let _ = write!(f, "INTF=(");
        if self.merrf_is_set() {
            let _ = write!(f, "MERRF, ");
        };
        if self.wakif_is_set() {
            let _ = write!(f, "WAKIF, ");
        };
        if self.errif_is_set() {
            let _ = write!(f, "ERRIF, ");
        };
        if self.tx2if_is_set() {
            let _ = write!(f, "TX2IF, ");
        };
        if self.tx1if_is_set() {
            let _ = write!(f, "TX1IF, ");
        };
        if self.tx0if_is_set() {
            let _ = write!(f, "TX0IF, ");
        };
        if self.rx1if_is_set() {
            let _ = write!(f, "RX1IF, ");
        };
        if self.rx0if_is_set() {
            let _ = write!(f, "RX0IF, ");
        };
        write!(f, ")")
    }
}

pub struct CanAddress {
    pub address: u32,
    extended: bool
}

impl CanAddress {
    pub fn extended(address: u32) -> CanAddress {
        CanAddress {
            address: address & (!(0b111u32 << 29)),
            extended: true
        }
    }

    pub fn standard(address: u16) -> CanAddress {
        CanAddress {
            address: (address & 0b111_1111_1111u16) as u32,
            extended: false
        }
    }

    pub fn is_extended(&self) -> bool {
        self.extended
    }
}

impl fmt::Debug for CanAddress {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let _ = write!(f, "CanAddress(");
        if self.is_extended() {
            let _ = write!(f, "ext:");
            let mut nibbles = [0u8; 8];
            let mut addr = self.address;
            for i in 0..=7usize {
                nibbles[i] = (addr & 0b1111) as u8;
                addr = addr >> 4;
            }
            for i in (0..=7usize).rev() {
                match i {
                    0 => { let _ = write!(f, "{:04b}", nibbles[i]); },
                    1..=6 => { let _ = write!(f, "{:04b}_", nibbles[i]); },
                    _ => { let _ = write!(f, "{:01b}_", nibbles[i]); }
                }
            }
        } else {
            let _ = write!(f, "std:");
        }
        write!(f, ")")
    }
}

pub struct McpTxBuf {
    pub txb_ctrl: u8,
    pub txb_sidh: u8,
    pub rts_cmd: u8
}

pub enum McpPriority {
    Low = 0,
    LowIntermediate = 1,
    HighIntermediate = 2,
    Highest = 3
}

pub enum McpAcceptanceFilter {
    RXF0 = 0,
    RXF1 = 1,
    RXF2 = 2,
    RXF3 = 3,
    RXF4 = 4,
    RXF5 = 5
}

pub enum McpReceiveBuffer {
    Buffer0,
    Buffer1
}

pub struct McpCanMessage {
    pub len: u8,
    pub data: [u8; 8],
    pub address: CanAddress,
    pub filter: McpAcceptanceFilter
}

impl fmt::Debug for McpCanMessage {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let _ = write!(f, "M: {:?} [{}] = [", self.address, self.len);
        for i in 0..self.len as usize {
            let _ = write!(f, "{:02x},", self.data[i]);
        }
        write!(f, "]")
    }
}

impl<E, SPI, CS> MCP25625<SPI, CS>
    where
        SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
        CS: OutputPin
{
    pub fn new(spi: SPI, cs: CS, one_cp: u32) -> Self {
        let mut ral = Mcp25625Ral::new(spi, cs, one_cp);
        ral.write_raw(&[Mcp25625Ral::<SPI, CS>::RESET_CMD]);
        Self {
            ral
        }
    }

    pub fn stat(&mut self) -> (McpOperationMode, McpStatInterruptFlags) {
        let canstat_reg = self.ral.read_reg(CANSTAT::address());
        let bits7_5 = (canstat_reg >> 5) & 0b111;
        let bits3_1 = (canstat_reg >> 1) & 0b111;
        let op_mode = match bits7_5 {
            0b000 => McpOperationMode::Normal,
            0b001 => McpOperationMode::Sleep,
            0b010 => McpOperationMode::Loopback,
            0b011 => McpOperationMode::ListenOnly,
            0b100 => McpOperationMode::Configuration,
            _ => McpOperationMode::Wrong
        };
        let i_flags = match bits3_1 {
            0b000 => McpStatInterruptFlags::NoInterrupt,
            0b001 => McpStatInterruptFlags::ErrorInterrupt,
            0b010 => McpStatInterruptFlags::WakeUpInterrupt,
            0b011 => McpStatInterruptFlags::TXB0Interrupt,
            0b100 => McpStatInterruptFlags::TXB1Interrupt,
            0b101 => McpStatInterruptFlags::TXB2Interrupt,
            0b110 => McpStatInterruptFlags::RXB0Interrupt,
            0b111 => McpStatInterruptFlags::RXB1Interrupt,
            _ => unreachable!()
        };
        (op_mode, i_flags)
    }

    pub fn change_mode(&mut self, to: McpOperationMode) -> Result<(), McpErrorKind> {
        if self.stat().0 != to {
            self.ral.write_reg(CANCTRL::address(), to.bits() << 5); // request new mode
            if self.stat().0 != to { // check
                return Err(McpErrorKind::ConfigRequestFailed);
            }
            Ok(())
        } else {
            Ok(())
        }
    }

    pub fn apply_config(&mut self, config: MCP25625Config) -> Result<(), McpErrorKind> {
        let sync_seg = 1u8;
        let tq_per_bit = sync_seg + config.prop_seg + config.ph_seg1 + config.ph_seg2;
        if !(tq_per_bit >= 5 && tq_per_bit <= 25) {
            return Err(McpErrorKind::WrongTqSum);
        }
        if !(config.brp <= 63) {
            return Err(McpErrorKind::WrongBrp);
        }
        if !(config.prop_seg >= 1 && config.prop_seg <= 8) {
            return Err(McpErrorKind::WrongPropSeg);
        }
        if !(config.ph_seg1 >= 1 && config.ph_seg1 <= 8) {
            return Err(McpErrorKind::WrongPhaseSeg1);
        }
        if !(config.ph_seg2 >= 2 && config.ph_seg2 <= 8) {
            return Err(McpErrorKind::WrongPhaseSeg2);
        }
        if !(config.sync_jump_width >= 1 && config.sync_jump_width <= 4) {
            return Err(McpErrorKind::WrongSJW);
        }
        if !(config.sync_jump_width <= config.ph_seg1 && config.sync_jump_width <= config.ph_seg2) {
            return Err(McpErrorKind::WrongSJW);
        }
        self.change_mode(McpOperationMode::Configuration)?;
        let cnf1: u8 = ((config.sync_jump_width-1) << 6) | config.brp;
        let cnf2: u8 = (1 << 7) | (1 << 6) | ((config.ph_seg1-1) << 3) | (config.prop_seg-1);
        let cnf3: u8 = config.ph_seg2 - 1;
        self.write_reg_verify(CNF1::address(), cnf1)?;
        self.write_reg_verify(CNF2::address(), cnf2)?;
        self.write_reg_verify(CNF3::address(), cnf3)?;
        Ok(())
    }

    pub fn masks_rxall(&mut self) -> Result<(), McpErrorKind> {
        let mask_regs: [u8; 8] = [
            0b0010_0000, 0b0010_0100,
            0b0010_0001, 0b0010_0101,
            0b0010_0010, 0b0010_0110,
            0b0010_0011, 0b0010_0111
        ];
        for m in mask_regs.iter() {
            self.write_reg_verify(*m, 0)?;
        }
        Ok(())
    }

    pub fn rx_configure(&mut self, rollover: bool, debug_rx: bool) -> Result<(), McpErrorKind> {
        let mut rxb0ctrl: u8 = 0;
        let mut rxb1ctrl: u8 = 0;
        if rollover {
            rxb0ctrl = rxb0ctrl | (1 << 2) | (1 << 1); // write to ro bit, so that verify succeeds
        }
        if debug_rx {
            rxb0ctrl = rxb0ctrl | (0b11 << 5);
            rxb1ctrl = rxb1ctrl | (0b11 << 5);
        }
        self.write_reg_verify(0b0110_0000, rxb0ctrl)?;
        self.write_reg_verify(0b0111_0000, rxb1ctrl)?;
        Ok(())
    }

    pub fn tec(&mut self) -> u8 {
        self.read_reg(0b0001_1100)
    }

    pub fn rec(&mut self) -> u8 {
        self.read_reg(0b0001_1101)
    }

    pub fn interrupt_flags(&mut self) -> McpInterruptFlags {
        let bits = self.ral.read_reg(0b0010_1100);
        McpInterruptFlags { bits }
    }

    pub fn enable_interrupts(&mut self) {
        self.ral.write_reg(0x2B, 0xFF);
    }

    pub fn reset_interrupt_flags(&mut self, mask: u8) {
        self.ral.bit_modify(0b0010_1100, mask, 0);
        // self.ral.write_reg(0b0010_1100, 0);
    }

    pub fn error_flags(&mut self) -> McpErrorFlags {
        let bits = self.ral.read_reg(0b0010_1101);
        McpErrorFlags { bits }
    }

    pub fn reset_error_flags(&mut self) {
        self.ral.write_reg(0b0010_1101, 0);
    }

    pub fn read_reg(&mut self, addr: u8) -> u8 {
        self.ral.read_reg(addr)
    }

    pub fn write_reg(&mut self, addr: u8, b: u8) {
        self.ral.write_reg(addr, b);
    }

    pub fn write_reg_verify(&mut self, addr: u8, b: u8) -> Result<(), McpErrorKind> {
        self.ral.write_reg(addr, b);
        let actual = self.ral.read_reg(addr);
        if actual == b {
            Ok(())
        } else {
            Err(McpErrorKind::RegVerifyError(addr))
        }
    }

    fn find_empty_txbuf(&mut self) -> Result<McpTxBuf, McpErrorKind> {
        let txb_n_ctrl: [u8; 3] = [0b0011_0000, 0b0011_0000 + 16, 0b0011_0000 + 32];
        let mut buf_idx = 666;
        for (i, txb_n_ctrl_addr) in txb_n_ctrl.iter().enumerate() {
            let txb_ctrl = self.read_reg(*txb_n_ctrl_addr);
            if txb_ctrl & (1 << 3) == 0 {
                buf_idx = i;
                break;
            }
        }
        if buf_idx <= 2 {
            Ok(McpTxBuf{
                txb_ctrl: txb_n_ctrl[buf_idx],
                txb_sidh: txb_n_ctrl[buf_idx] + 1,
                rts_cmd: Mcp25625Ral::<SPI, CS>::RTS_CMD | (1u8 << buf_idx as u8)
                //txb_sidl: 0x32 + buf_idx as u8 * 0x10,
                //txb_eid8: 0x33 + buf_idx as u8 * 0x10,
                //txb_eid0: 0x34 + buf_idx as u8 * 0x10,
                //txb_dlc:  0x35 + buf_idx as u8 * 0x10,
                //txb_data: 0x36 + buf_idx as u8 * 0x10
            })
        } else {
            Err(McpErrorKind::NoTxSlotsAvailable)
        }
    }

    pub fn send(&mut self, addr: CanAddress, data: &[u8], _: McpPriority) -> Result<(), McpErrorKind> {
        if data.len() > 8 {
            return Err(McpErrorKind::TooBig);
        }
        let buf = self.find_empty_txbuf()?;
        let mut sidl: u8 = ((addr.address & 0b111) as u8) << 5;
        if addr.is_extended() {
            sidl = sidl | (1 << 3) | ((addr.address >> 27 & 0b11) as u8);
        }
        let eid8 = (addr.address >> 19 & 0xff) as u8;
        let eid0 = (addr.address >> 11 & 0xff) as u8;
        let sidh = (addr.address >> 3 & 0xff) as u8;
        let dlc = data.len() as u8 & 0b1111;
        //let ctrl = (1 << 3) | ((priority as u8) & 0b11);
        let buf_config = [sidh, sidl, eid8, eid0, dlc];
        self.ral.write_many2(buf.txb_sidh, &buf_config, data);
        self.ral.write_raw(&[buf.rts_cmd]);
        Ok(())
    }

    /// Read RX buffer and clear INTF flags automatically on cs pin raise
    pub fn receive(&mut self, buffer: McpReceiveBuffer) -> McpCanMessage {
        let block_start_addr = match buffer {
            McpReceiveBuffer::Buffer0 => McpFastRxRead::RXB0SIDH, // sidh addr
            McpReceiveBuffer::Buffer1 => McpFastRxRead::RXB1SIDH
        };
        let mut buf = [0u8; 13];
        self.ral.read_rxbuf(block_start_addr, &mut buf);
        let sidh = buf[0];
        let sidl = buf[1];
        let eid8 = buf[2];
        let eid0 = buf[3];
        let dlc = buf[4];

        let is_extended = sidl & (1 << 3) != 0;
        let address = if is_extended {
            CanAddress::extended((((sidl & 0b11) as u32) << 27) | ((eid8 as u32) << 19) | ((eid0 as u32) << 11) | ((sidh as u32) << 3) | ((sidl >> 5) as u32))
        } else {
            CanAddress::standard((sidh as u16) | ((sidl >> 5) as u16))
        };

        let mut data = [0u8; 8];
        data.copy_from_slice(&buf[5..13]);
        let mut data_len = dlc & 0b1111;
        if data_len > 8 {
            data_len = 8;
        }
        McpCanMessage {
            len: data_len,
            data,
            address,
            filter: McpAcceptanceFilter::RXF0
        }
    }

    pub fn led_on(&mut self) {
        self.ral.write::<BFPCTRL, _>(|w| w
            .b0fm().output()
            .b0fs().high()
            .b0fe().enable() );
    }

    pub fn led_off(&mut self) {
        self.ral.write::<BFPCTRL, _>(|w| w
            .b0fm().output()
            .b0fs().low()
            .b0fe().enable() );
    }
}