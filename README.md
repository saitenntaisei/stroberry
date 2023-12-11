# DangoromouseZero

This is a software for micromouse powered by HAL librarary. [Haradware](https://github.com/dangorogoro/DangoromouseZero-PCB) is designed by [@dangorogoro](https://github.com/dangorogoro).  

## Getting Started

### Prerequisites

This repository require following sofware.

- gcc-arm-none-eabi
- openocd
- Coretex-Debug

I use [asdf](https://asdf-vm.com/) to manage gcc-arm-none-ebi.

```console
asdf plugin add gcc-arm-none-eabi
asdf install gcc-arm-none-eabi latest
asdf global gcc-arm-none-eabi latest
sudo apt update
sudo apt install openocd
sudo apt install build-essential
```

### Installing

Please modify path in task.json.

## Deployment

Please be sure that this use ST-Link driver to wrtie program in the board.

## Authors

- **saitenntaisei**

## License

This project is licensed under the MIT License.
