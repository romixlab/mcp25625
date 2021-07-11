# mcp25625

Full support for tranmission, recepetion and filtering with convenient API.

Define some types to keep everything clean:
```rust
pub type Mcp25625Sck = PB3<Analog>;
pub type Mcp25625Miso = PA11<Analog>;
pub type Mcp25625Mosi = PA12<Analog>;
pub type Mcp25625IrqUninit = PA10<Analog>;
pub type Mcp25625Cs = PA9<Output<PushPull>>;
pub type Mcp25625Spi = hal::pac::SPI1;
pub type Mcp25625Instance = mcp25625::MCP25625<hal::spi::Spi<Mcp25625Spi, (Mcp25625Sck, Mcp25625Miso, Mcp25625Mosi)>, Mcp25625Cs>;
```

Also define filters to use:
```rust
const MOTOR_DRIVE_ID: FrameId = FrameId::new_extended(0x1).unwrap();
const EMERGENCY_STOP_ID: FrameId = FrameId::new_standard(0x0).unwrap();
```

Setup filters and configure:
```rust
fn mcp25625_configure(mcp25625: &mut config::Mcp25625Instance) -> Result<(), McpErrorKind> {
    let filters_buffer0 = FiltersConfigBuffer0 {
        mask: FiltersMask::AllExtendedIdBits,
        filter0: config::MOTOR_DRIVE_ID,
        filter1: None
    };
    let filters_buffer1 = FiltersConfigBuffer1 {
        mask: FiltersMask::OnlyStandardIdBits,
        filter2: config::EMERGENCY_STOP_ID,
        filter3: None,
        filter4: None,
        filter5: None,
    };
    let filters_config = FiltersConfig::Filter(filters_buffer0, Some(filters_buffer1));
    let mcp_config = MCP25625Config {
        brp: 0, // Fosc=16MHz
        prop_seg: 3,
        ph_seg1: 2,
        ph_seg2: 2,
        sync_jump_width: 2,
        rollover_to_buffer1: true,
        filters_config,
        // filters_config: FiltersConfig::ReceiveAll,
        operation_mode: McpOperationMode::Normal
    };
    mcp25625.apply_config(mcp_config)?;
    Ok(())
}
```
Unused filter slots are filled with the latest one used. So in this example `filter1` = `filter0` and `filter3-5` = `filter2`.

Enable interrupts:
```rust
mcp25625.enable_interrupts(0b0001_1111);
```

Receive a frame:
```rust
let intf = mcp25625.interrupt_flags();
if intf.rx0if_is_set() {
    let frame: vhrdcan::RawFrame = mcp25625.receive(McpReceiveBuffer::Buffer0);
    println!("{:?}", frame);
}
// Check Buffer1 also depending on the configuration
```

Send a frame:
```rust
// Create RawFrameRef if you do not want to make unnecesarry copies
// or use Frame::as_raw_frame_ref() or RawFrame::as_raw_frame_ref()
let data = [1, 2, 3];
let frame = vhrdcan::RawFrameRef {
    id: vhrdcan::FrameId::new_standard(0x10).unwrap(),
    data: &data
};
match mcp25625.send(frame) {
    Ok(_) => {
        println!("Frame queued for transmission");
    }
    Err(e) => {
        println!("Frame transmission error: {:?}", e);
    }
}
```

For more advanced example see [this](https://github.com/vhrdtech/vhrdbms/blob/main/l031c6_gcarrier/src/tasks/canbus.rs) file with hot reinit, interrupts and proper queues for transmission and reception.
