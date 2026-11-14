## Feature removals

### Removal of self-test interfaces

TF-PSA-Crypto 1.0 does not provide a ready-made self-test interface; one may be added in a future version of the library.

If you need self-tests for compliance, you may perform them by invoking normal API functions with sample data.

As a consequence, the compilation option `MBEDTLS_SELF_TEST` does not provide direct benefits in TF-PSA-Crypto 1.0. However, it allows the sample program `programs/test/selftest.c` in Mbed TLS to run self tests of cryptographic mechanisms.

### Removed hardware support

Acceleration for VIA Padlock (`MBEDTLS_PADLOCK_C`) is no longer provided.

The deprecated and incomplete support for dynamic registration of secure element drivers (`MBEDTLS_PSA_CRYPTO_SE_C`) has been removed. Use compile-time secure element drivers instead.

See also [the removal of ALT interfaces](#removal-of-alternative-cryptographic-module-implementations).

### Removed obsolete cryptographic mechanisms

Some obsolescent cryptographic mechanisms have been removed:

* The library no longer supports DES (including 3DES). All supported block ciphers now have 128-bit blocks. As a consequence, the PKCS12 module, which provided the obsolete PBE mode for private key encryption that does not support stronger ciphers, has been removed.
* The library no longer supports elliptic curves whose size is 224 bits or less. The following curves are no longer supported: secp192r1, secp192k1, secp224k1, secp224r1. Use larger curves such as secp256r1.

### Removed obsolete PSA functions

Some PSA Crypto API functions dating back from before the 1.0 version of the API, or that were experimental, have been removed:

* `psa_open_key()`, `psa_close_key()`, and auxiliary functions and macros related to handles. Persistent keys are opened implicitly since Mbed TLS 2.25.
* `psa_set_key_domain_parameters()`, `psa_get_key_domain_parameters()` and related macros. This feature was primarily intended to support custom finite-field Diffie-Hellman (FFDH) groups, but this was never implemented. To generate an RSA key with a custom public exponent, use `psa_generate_key_custom()`, introduced in Mbed TLS 3.6.1.
* `psa_generate_key_ext()`, `psa_key_derivation_output_key_ext` and related types and macros. Use `psa_generate_key_custom()` or `psa_key_derivation_output_key_custom()` instead.
