## PSA as the only cryptography API

The PSA Crypto API is now the only API for cryptographic primitives.

For general guidance on migrating to the PSA Crypto API, consult the
[PSA transition guide](../psa-transition.md). Note that most of the suggested migrations also work in the Mbed TLS 3.6 long-time support branch, provided that the library is configured suitably (`MBEDTLS_USE_PSA_CRYPTO` and `MBEDTLS_PSA_CRYPTO_CONFIG` enabled).

### Impact on application code

The PK module uses PSA for cryptographic operations. This corresponds to the behavior of Mbed TLS 3.x when `MBEDTLS_USE_PSA_CRYPTO` is enabled. In effect, `MBEDTLS_USE_PSA_CRYPTO` is now always enabled.

`psa_crypto_init()` must be called before performing any cryptographic operation.

A few functions take different parameters to migrate them to the PSA API. See “[Function prototype changes](#function-prototype-changes)”.

### No random generator instantiation

Formerly, applications using various cryptographic features needed to provide a random generator, generally by instantiating an entropy context (`mbedtls_entropy_context`) and a DRBG context (`mbedtls_ctr_drbg_context` or `mbedtls_hmac_drbg_context`). This is no longer necessary, or possible. All features that require a random generator (RNG) now use the one provided by the PSA subsystem.

Instead, applications that use random generators or keys (even public keys) need to call `psa_crypto_init()` before any cryptographic operation or key management operation.

See also [function prototype changes](#function-prototype-changes), many of which are related to the move from RNG callbacks to a global RNG.

### Impact on the library configuration

The choice of supported cryptographic mechanisms is now based on `PSA_WANT_xxx` macros instead of legacy configuration macros such as `MBEDTLS_RSA_C`, `MBEDTLS_PKCS1_V15`, etc. This corresponds to the behavior of Mbed TLS 3.x when `MBEDTLS_PSA_CRYPTO_CONFIG` is enabled. In effect, `MBEDTLS_PSA_CRYPTO_CONFIG` is now always enabled.

For information on which configuration macros are affected and their new PSA equivalent, consult the [PSA transition guide](../psa-transition.md).

See also the section [“Random number generation configuration”](#random-number-generation-configuration).

### No direct access to specific algorithms

All modules that are specific to a particular cryptographic mechanism have been removed from the API. There are a few exceptions, for some mechanisms that are not yet present in the PSA API: `mbedtls/lms.h` and `mbedtls/nist_kw.h` remain part of the API.

The high-level legacy module `mbedtls/cipher.h` has also been removed. The high-level legacy modules `mbedtls/md.h` and `mbedtls/pk.h` remain present with reduced functionality (see “[Changes to MD and PK](#changes-to-md-and-pk)”). TF-PSA-Crypto also retains non-PSA interfaces for data formats, platform support and miscellaneous utility functions.

In full detail, the following header files, and their former content, are no longer available.

```
everest/Hacl_Curve25519.h
everest/everest.h
everest/kremlib.h
everest/kremlib/*.h
everest/kremlin/*.h
everest/kremlin/internal/*.h
everest/vs2013/*.h
everest/x25519.h
mbedtls/aes.h
mbedtls/aria.h
mbedtls/bignum.h
mbedtls/block_cipher.h
mbedtls/camellia.h
mbedtls/ccm.h
mbedtls/chacha20.h
mbedtls/chachapoly.h
mbedtls/cipher.h
mbedtls/cmac.h
mbedtls/ctr_drbg.h
mbedtls/des.h
mbedtls/dhm.h
mbedtls/ecdh.h
mbedtls/ecdsa.h
mbedtls/ecjpake.h
mbedtls/ecp.h
mbedtls/entropy.h
mbedtls/gcm.h
mbedtls/hkdf.h
mbedtls/hmac_drbg.h
mbedtls/md5.h
mbedtls/pkcs12.h
mbedtls/pkcs5.h
mbedtls/poly1305.h
mbedtls/ripemd160.h
mbedtls/rsa.h
mbedtls/sha1.h
mbedtls/sha256.h
mbedtls/sha3.h
mbedtls/sha512.h
```

If your application was using functions from these headers, please see
[`docs/psa-transition.md`](../psa-transition.md) for ways to migrate to
the PSA API instead.

Some of the associated types still need to be visible to the compiler. For example, `mbedtls_aes_context` is used to define `psa_cipher_operation_t`. These types are still available when building application code, but we recommend that you no longer use them directly. The structure, the semantics and even the existence of these types may change without notice.

### Other removed functions related to low-level cryptography APIs

The functions `mbedtls_ecc_group_to_psa()` and `mbedtls_ecc_group_from_psa()` have been removed. They are no longer meaningful since the low-level representation of elliptic curve groups is no longer part of the API.

### Removal of alternative cryptographic module implementations

TF-PSA-Crypto no longer supports replacing a whole cryptographic module or an individual cryptographic function by defining a macro `MBEDTLS_xxx_ALT` and providing a custom implementation of the same interface. Instead, use PSA transparent drivers (i.e. accelerator drivers).

The PK module no longer supports `MBEDTLS_PK_RSA_ALT`. Instead, for opaque keys (RSA or otherwise), use PSA opaque drivers.
