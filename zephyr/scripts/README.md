# Describing Pin States #

This document explains all steps required to support pin states' generation for Espressif SoCs supported on the Zephyr RTOS. Pin states are consumed by the `pinctrl` subsystem.

It is initially meant to be a reference to `hal_espressif` contributors only, though the outcomes of this work (pin states) can be overlayed by Zephyr RTOS users basing its applications on our SoCs.

## Supporting a new SoC ##

When supporting a new SoC, some conventions must be followed for proper pin state generation:

1. Create a folder under `dts/espressif` named with the lowercased SoC suffix (e.g., `32` for `ESP32`, `32c3` for `ESP32C3`, etc.).
2. Create a `pinctrl.yaml` file under the new folder. This file will describe all properties required for state generation as explained on next section.

In order to illustrate the above steps, let us assume we are supporting `ESP32S2` for the first time. The following commands will do the job:

```sh
$ cd dts/espressif
$ mkdir 32s2
$ vim pinctrl.yaml
```

## Filling `pinctrl.yaml` ##

It is easier to explain through an example, for this purpose, let us take `ESP32`'s UART_0 to illustrate: 

```yaml
uart0:
  tx:
    func: [out]
    sigo: u0txd_out
    gpio:
      g0:
        pins: [[0, 21], [26, 45]]
  rx:
    func: [in]
    sigi: u0rxd_in
    gpio:
      g0:
        pins: [[0, 21], [26, 45]]
        bias: pull_up
      g1:
        pins: [46]
```

Compare the following description of the required fields in a `pinctrl.yaml` file with the above snippet. The bullet indentation follows the original file indentation to ease visual comparisons. 

- **`{peripheral}`** (required): concatenation of the peripheral name and its instance number (respectively, `uart` and `0` from the snippet above).
- **`{periph_sig}`** (required): freely-named field which depends on the target peripheral, `tx` and `rx` above are examples of `periph_sig`.
    - **`func`** (required): array of one or more of the following string values: `out`, `out_low`, `out_high` and `in`.
    - **`sigo`** (optional): output signal, required if `func` is one of `out`, `out_low` or `out_high`.
    - **`sigi`** (optional): input signal, required if `func` is `in`.
    - **`gpio`** (required):
        - **`g{N}`** (required): represents an IO group, where `N` is a group index. At least one group - `g0` - is required per gpio attribute. Other groups must be numbered sequentially. An IO group allows to apply some pin attributes in a subset of pins within the same peripheral, if necessary.
            - **`pins`** (required): an array composed of __integers__ and/or __2-sized arrays of integers__. Integer elements represent a pin number (e.g., `1` means `GPIO1`) while a 2-sized array represents a pin number range (e.g., `[0, 21]` means `GPIO0`, `GPIO1`, ... , `GPIO21`).
            - **`bias`** (optional): one of `pull_up` or `pull_down`.
            - **`outm`** (optional): output mode. One of `open_drain` or `push_pull`.

---
**NOTES**

- The values for the fields `sigi` and `sigo` must be borrowed from `include/dt-bindings/pinctrl/esp{soc_id}-gpio-sigmap.h` file. Drop the `ESP_` prefix and lowercase the result (e.g., `ESP_U0TXD_OUT` becomes `u0txd_out`).
- If a GPIO group is formed by either only one pin number or only one pin number range, in any case, enclosing in `[]` is still required, remember, the script expects an array. Failing to do so, may lead to unintended states generation. For example, `[0, 19]` is **not** the same as `[[0, 19]]`. The last generates states for a pin range, while the former generates states for `GPIO0` and `GPIO19` only.
- The fields enclosed in `{}` should be replaced by peripheral-specific names.
- Mind the indentation.

---

## Generating pin states ##

From `hal_espressif`'s root folder, run the script (here, using `ESP32-S2` as target):

```sh
$ ./zephyr/scripts/esp_genpinctrl.py -p dts/espressif/32s2
```

The script looks for the `pinctrl.yaml` file in the target path and will generate the equivalent `.dtsi` file in the same folder. The script will also append the leaf of the path to the generated file, yielding a `esp{soc_id}-pinctrl.dtsi` file. In the above command, it will lead to the generation of `esp32s2-pinctrl.dtsi`.

Now the pin states are ready to be imported from the device tree.
