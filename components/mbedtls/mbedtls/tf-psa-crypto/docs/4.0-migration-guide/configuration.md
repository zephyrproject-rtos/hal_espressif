## Compile-time configuration

### Configuration file split

Whether you are using TF-PSA-Crypto as a standalone project or as part of Mbed TLS, all the configuration options that are relevant to TF-PSA-Crypto must be configured in one of its configuration files, namely:

* `TF_PSA_CRYPTO_CONFIG_FILE`, if set on the preprocessor command line;
* otherwise `<psa/crypto_config.h>`;
* additionally `TF_PSA_CRYPTO_USER_CONFIG_FILE`, if set.

The macros `MBEDTLS_PSA_CRYPTO_CONFIG_FILE` and `MBEDTLS_PSA_CRYPTO_USER_CONFIG_FILE` are no longer recognized. Use `TF_PSA_CRYPTO_CONFIG_FILE` and `TF_PSA_CRYPTO_USER_CONFIG_FILE` instead

Configuration options that are relevant to X.509 or TLS should still be set in the Mbed TLS configuration file (`MBEDTLS_CONFIG_FILE` or `<mbedtls/mbedtls_config.h>`, plus `MBEDTLS_USER_CONFIG_FILE` if it is set). However, you can define all options in the crypto configuration, and Mbed TLS will pick them up.

Generally speaking, the options that must be configured in TF-PSA-Crypto are:

* options related to platform settings;
* options related to the choice of cryptographic mechanisms included in the build;
* options related to the inner workings of cryptographic mechanisms, such as size/memory/performance compromises;
* options related to crypto-adjacent features, such as ASN.1 and Base64.

See `include/psa/crypto_config.h` in TF-PSA-Crypto and `include/mbedtls/mbedtls_config.h` in Mbed TLS for details.

Notably, `<psa/crypto_config.h>` is no longer limited to `PSA_WANT_xxx` options.

Note that many options related to cryptography have changed; this is covered in other sections of this document.

### Split of `build_info.h` and `version.h`

TF-PSA-Crypto has a header file `<tf-psa-crypto/build_info.h>` which includes the configuration file and provides the adjusted configuration macros, similar to `<mbedtls/build_info.h>` in Mbed TLS.

TF-PSA-Crypto exposes its version through `<tf-psa-crypto/version.h>`, similar to `<mbedtls/version.h>` in Mbed TLS.

### Removal of `check_config.h`

The header `mbedtls/check_config.h` is no longer present. Including it from user configuration files was already obsolete in Mbed TLS 3.x, since it enforces properties of the configuration as adjusted by `mbedtls/build_info.h`, not properties that the user configuration is expected to meet.
