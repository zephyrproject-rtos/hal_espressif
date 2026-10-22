ESP32 PSA Crypto drivers for Zephyr
###################################

Transparent PSA Crypto drivers for the ESP32 cryptographic accelerators, plugged
into TF-PSA-Crypto through Zephyr's ``CONFIG_TF_PSA_CRYPTO_DISPATCH_DIR`` and
``CONFIG_TF_PSA_CRYPTO_DRIVER_CONFIG_FILE``.

Drivers
*******

``esp_psa_sha.c``
  Hash driver for the SHA accelerator: SHA-1, SHA-224 (where the SoC has it) and
  SHA-256. Every other algorithm is declined and handled by TF-PSA-Crypto's
  built-in implementation, which stays compiled in. The hashing logic is
  ESP-IDF's (``components/mbedtls/port/psa_driver/esp_sha/core/``); this file is
  the PSA side of it, with the operation context stored inside
  ``psa_hash_operation_t`` (no heap) and the output size checked before
  anything is written.

``esp_sha_port.c``
  Zephyr implementation of the engine primitives the ESP-IDF cores call
  (``components/mbedtls/port/include/sha/sha_core.h``): a ``k_mutex`` around the
  engine, the clock from the ``sha`` devicetree node, block mode only. A block
  is started only once the engine is idle, in the order the TRM gives (write
  the words, wait for BUSY, START/CONTINUE); ``sha_hal_hash_block()`` does not
  wait, and on the ESP32-P4 a CONTINUE on a busy engine gives a wrong digest.

``shim/mbedtls/esp_config.h``
  Stand-in for ESP-IDF's Mbed TLS configuration, seen only by the driver and the
  ESP-IDF cores. It includes ``<psa/crypto.h>`` before defining the macros the
  cores test, so the PSA structures keep the layout every other translation
  unit sees.

``include/``
  ``esp_psa_driver_config.h`` (the driver configuration file),
  ``esp_psa_sha_ctx.h`` (the opaque operation context, free of HAL headers
  because it reaches every user of ``<psa/crypto.h>``) and ``esp_psa_sha.h``
  (the driver entry points).

Dispatch
********

``dispatch/`` is TF-PSA-Crypto's ``dispatch/`` directory with the driver's
branches added to the six ``psa_driver_wrapper_hash_*`` functions, its context
to the hash union and its ID to the driver enumeration. TF-PSA-Crypto's
driver-wrapper generator does not emit hash entry points (they are hand-written
in its template), so the copy is produced by ``gen_dispatch.py``:

.. code-block:: console

   python3 gen_dispatch.py <tf-psa-crypto>/dispatch dispatch

Regenerate it whenever the TF-PSA-Crypto revision in Zephyr's ``west.yml``
changes. The script fails if any insertion point is not found exactly once.

Source revision: TF-PSA-Crypto ``765af96a20a2``, the revision Zephyr ``main`` pins.

Licence: the TF-PSA-Crypto files are dual-licensed Apache-2.0 OR GPL-2.0-or-later;
they are used here under Apache-2.0.
