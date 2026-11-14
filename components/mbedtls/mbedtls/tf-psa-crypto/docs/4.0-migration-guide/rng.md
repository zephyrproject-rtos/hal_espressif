## Random number generation configuration

TF-PSA-Crypto no longer exposes the internals of the PSA random number generator. The entropy, CTR_DRBG, and HMAC_DRBG modules from Mbed TLS 3.6 are now for internal use only. As a result, their configuration has been updated, both to simplify them and to prepare for PSA entropy and random number generation drivers, which will be introduced in a future minor release.

The overall structure of the random number generator in TF-PSA-Crypto remains the same as in Mbed TLS 3.x. It consists of either:
* an external random number generator (when `MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG` is enabled), or
* a deterministic random number generator (CTR_DRBG or HMAC_DRBG) seeded with entropy by the entropy module.

### Entropy configuration

TF-PSA-Crypto does not expose an entropy interface to applications. The entropy module of Mbed TLS 3.6 is now for internal use only. As a consequence, its configuration has changed, both to simplify it and to prepare for PSA entropy drivers which will be added in a future minor release.

#### If you have a fast cryptographic-quality external random generator

If you have a fast, cryptographic-quality source of random data, enable `MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG` and provide the callback function `mbedtls_psa_external_get_random()`. This is unchanged since Mbed TLS 2.26.

#### If the built-in entropy source works for you

TF-PSA-Crypto, like Mbed TLS, recognizes the system entropy sources on some popular operating systems, and enables them by default. If you are writing your own configuration, you now need to define the option `MBEDTLS_PSA_BUILTIN_GET_ENTROPY`. (This is different from Mbed TLS 2.x and 3.x, where the built-in entropy sources were included unless the option `MBEDTLS_NO_PLATFORM_ENTROPY` was defined.)

#### If you have a custom entropy source

To provide a custom entropy source, disable `MBEDTLS_PSA_BUILTIN_GET_ENTROPY` and enable `MBEDTLS_PSA_DRIVER_GET_ENTROPY`. You will need to provide the callback function `mbedtls_platform_get_entropy()`. See [“Custom entropy collector”](#custom-entropy-collector) below for more details.

#### If you have no entropy source

If your hardware does not have any entropy source, you can use a non-volatile RNG seed which is injected from a trusted external source during device manufacturing or provisioning. Enable the option `MBEDTLS_ENTROPY_NV_SEED`, which is unchanged from previous versions of the library.

In TF-PSA-Crypto, when the non-volatile seed is the only (pseudo) entropy source, you also need to enable the option `MBEDTLS_ENTROPY_NO_SOURCES_OK`.

### Comparison of entropy source configurations

TF-PSA-Crypto 1.0 supports the same entropy sources as Mbed TLS 3.6, but the way to configure them has changed.

* The negative option `MBEDTLS_NO_PLATFORM_ENTROPY` to disable the default entropy collector for Unix-like and Windows platforms no longer exists. It has been replaced by the positive option `MBEDTLS_PSA_BUILTIN_GET_ENTROPY`, which is enabled by default.
* The option `MBEDTLS_ENTROPY_HARDWARE_ALT`, which allows you to provide a custom entropy collector, has been renamed to `MBEDTLS_PSA_DRIVER_GET_ENTROPY`. This replaces `MBEDTLS_ENTROPY_HARDWARE_ALT`. The callback has a different name and prototype as described in “[Custom hardware collector](#custom-entropy-collector)”.
* The option `MBEDTLS_ENTROPY_NV_SEED` to enable a non-volatile seed is unchanged. However, if this is your only entropy source, you must now enable the new option `MBEDTLS_ENTROPY_NO_SOURCES_OK`.

The following table describes common configurations.

<table>
  <tr valign="top">
    <th align="left">Configuration</th>
    <th align="left">Mbed TLS 3.6</th>
    <th align="left">TF-PSA-Crypto 1.0</th>
  </tr>

  <tr valign="top">
    <td><strong>Unix, Linux, Windows</strong></td>
    <td>(default)</td>
    <td>(default)</td>
  </tr>

  <tr valign="top">
    <td><strong>Embedded platform</strong></td>
    <td>
      <pre>
#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_ENTROPY_HARDWARE_ALT
      </pre>
    </td>
    <td>
      <pre>
#undef MBEDTLS_PSA_BUILTIN_GET_ENTROPY
#define MBEDTLS_PSA_DRIVER_GET_ENTROPY
      </pre>
    </td>
  </tr>

  <tr valign="top">
    <td><strong>Fast external crypto RNG</strong></td>
    <td>
      <pre>
#define MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG
#undef MBEDTLS_ENTROPY_C
#undef MBEDTLS_CTR_DRBG_C
      </pre>
    </td>
    <td>
      <pre>
#define MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG
#undef MBEDTLS_ENTROPY_C
#undef MBEDTLS_CTR_DRBG_C
      </pre>
    </td>
  </tr>

  <tr valign="top">
    <td><strong>NV seed only</strong></td>
    <td>
      <pre>
#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_ENTROPY_NV_SEED
      </pre>
    </td>
    <td>
      <pre>
#undef MBEDTLS_PSA_BUILTIN_GET_ENTROPY
#define MBEDTLS_ENTROPY_NV_SEED
#define MBEDTLS_ENTROPY_NO_SOURCES_OK
      </pre>
    </td>
  </tr>

  <tr valign="top">
    <td><strong>No entropy at all</strong></td>
    <td>
      <pre>
#define MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES
      </pre>
    </td>
    <td>
      <em>not supported</em>
    </td>
  </tr>
</table>

#### Custom entropy collector

The custom entropy collector callback function has changed, to make it match the upcoming PSA entropy driver specification.

Formerly, the callback was enabled by `MBEDTLS_ENTROPY_HARDWARE_ALT` and had the following prototype:
```c
// from <entropy_poll.h>
int mbedtls_hardware_poll(void *data, unsigned char *output, size_t len,
                          size_t *olen);
```

The new callback is enabled by `MBEDTLS_PSA_DRIVER_GET_ENTROPY` and has the following prototype:
```c
```

to:

```c
// from <mbedtls/platform.h>
int mbedtls_platform_get_entropy(psa_driver_get_entropy_flags_t flags,
                                 size_t *estimate_bits,
                                 unsigned char *output, size_t output_size);
```

The `data` parameter was previously always `NULL`, and has been removed.

The new parameter `flags` is a bit-mask of flags that allows the caller to request special behaviors, such as avoiding blocking. The callback should return `PSA_ERROR_NOT_SUPPORTED` if it sees a flag that it does not support. As of TF-PSA-Crypto 1.0, `flags` is always 0.

The former callback could return less entropy than expected by only filling part of the buffer, and setting `*olen` to a value that is less than `output_size`. The new callback does not have an `olen` parameter, and the caller now reads the whole buffer. The new parameter `estimate_bits` is intended to allow the callback to report that it has accumulated less entropy than expected. However, this is not supported yet in TF-PSA-Crypto 1.0.

The new output parameter `estimate_bits` is the amount of entropy that the callback has placed in the output buffer. As of TF-PSA-Crypto 1.0, the output must have full entropy, thus `estimate_bits` must be equal to `8 * output_size`. A future version of TF-PSA-Crypto will allow entropy sources to report smaller amounts.

To indicate that entropy is not currently available, the legacy error code `MBEDTLS_ERR_ENTROPY_SOURCE_FAILED` has been replaced by `PSA_ERROR_INSUFFICIENT_ENTROPY`.

#### Removed entropy options

The option `MBEDTLS_ENTROPY_MIN_HARDWARE` has been removed. The entropy module requests the amount that it needs for the chosen security strength.

The option `MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES` has removed since it is no longer meaningful, now that the entropy module is private. TF-PSA-Crypto 1.0 does not support platforms without an entropy source. This ability will be reintroduced in a future minor release.

#### PSA entropy injection

The configuration option `MBEDTLS_PSA_INJECT_ENTROPY` has been removed. TF-PSA-Crypto 1.0 does not provide a way to store an entropy seed in the key store. This will be reimplemented in a future minor version.

### Configuration of the DRBG

As in previous versions of Mbed TLS, the PSA random generator in TF-PSA-Crypto uses CTR_DRBG with AES if `MBEDTLS_CTR_DRBG_C` is enabled, and HMAC_DRBG otherwise (requiring `MBEDTLS_HMAC_DRBG_C` to be enabled).

The DRBG modules are not exposed directly, they are only used internally.

In addition to the choice of DRBG module and entropy sources, the built-in random number generator is now configured through only three options:

* `MBEDTLS_PSA_CRYPTO_RNG_HASH`: Selects the hash algorithm used by the entropy and HMAC_DRBG modules. This option replaces both `MBEDTLS_PSA_HMAC_DRBG_MD_TYPE` and `MBEDTLS_ENTROPY_FORCE_SHA256`.
* `MBEDTLS_PSA_RNG_RESEED_INTERVAL`: Sets the reseed interval for both CTR_DRBG and HMAC_DRBG. It replaces `MBEDTLS_CTR_DRBG_RESEED_INTERVAL` and `MBEDTLS_HMAC_DRBG_RESEED_INTERVAL`.
* `MBEDTLS_PSA_CRYPTO_RNG_STRENGTH`: Specifies the security strength in bits. The default is 256 bits. If you previously enabled `MBEDTLS_CTR_DRBG_USE_128_BIT_KEY` in Mbed TLS 3.6, you should now set `MBEDTLS_PSA_CRYPTO_RNG_STRENGTH` to 128, although this is not recommended.

The option `MBEDTLS_ENTROPY_C` has been removed. The entropy module is now automatically enabled when `MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG` is disabled.
The following Mbed TLS 3.6 configuration options have been removed without any counterpart in TF-PSA-Crypto. Their corresponding APIs were also removed, making these options no longer relevant:
`MBEDTLS_ENTROPY_MAX_GATHER`, `MBEDTLS_ENTROPY_MAX_SOURCES`, `MBEDTLS_CTR_DRBG_ENTROPY_LEN`, `MBEDTLS_CTR_DRBG_MAX_INPUT`, `MBEDTLS_CTR_DRBG_MAX_REQUEST`, `MBEDTLS_CTR_DRBG_MAX_SEED_INPUT`, `MBEDTLS_HMAC_DRBG_MAX_INPUT`, `MBEDTLS_HMAC_DRBG_MAX_REQUEST`, and `MBEDTLS_HMAC_DRBG_MAX_SEED_INPUT`.
