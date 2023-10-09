# Aqua Sense - Smart Water Faucet Monitor

## Introduction
Welcome to the repository of "Aqua Sense," a smart water faucet monitor developed as part of a university project. Aqua Sense is designed with the intention to promote judicious water usage by providing real-time feedback via a straightforward and intuitive interface affixed to household faucets.

## Technology Stack
The Aqua Sense application is implemented using the [Rust](https://www.rust-lang.org/) programming language and is engineered to operate seamlessly with the Raspberry Pi Pico.

## Installation and Running

Ensure you adhere to the following steps for a hassle-free setup and execution of Aqua Sense on your Raspberry Pi Pico.

### Prerequisites

1. **Install elf2uf2-rs:**
   Convert ELF files to UF2 format, suitable for flashing onto the Raspberry Pi Pico.
   ```shell
   cargo install elf2uf2-rs --locked
   ```
2. **Add the ARM Cortex-M thumb target:**
    ```shell
    rustup target add thumbv6m-none-eabi
    ```
3. **Install probe-run:**
    ```shell
    cargo install probe-run
    ```

### Deploying Aqua Sense to Raspberry Pi Pico
Once all prerequisites are set, deploy the Aqua Sense code to your Raspberry Pi Pico with:

```shell
cargo run --bin aqua_sense.rs
```

## License
This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.
