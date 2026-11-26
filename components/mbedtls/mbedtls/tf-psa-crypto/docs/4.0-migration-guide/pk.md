## PK module

The PK module for asymmetric cryptography is still present in TF-PSA-Crypto 1.x, but with a somewhat reduced scope compared to Mbed TLS. The main goal of PK is now to parse and format key pairs and public keys. PK also retains signature functions, to facilitate the transition from previous versions. The main differences compared to Mbed TLS 3.6 and below are:

* PK objects no longer expose the underlying representation. It is now unspecified in most cases whether a PK object stores key material directly, or a PSA key identifier.
* PK objects no longer have any policy attached to them. This mostly affects RSA keys, which no longer “remember” whether they are meant to be used with PKCS#1v1.5 or PSS/OAEP.

### Changes to PK metadata

The type `mbedtls_pk_type_t` has been removed from the API. This type could convey different kinds of information, depending on where it was used:

* Information about the representation of a PK context (distinguishing between transparent and opaque objects containing the same type of key). This information is no longer exposed.

* Which signature algorithm to use (distinguishing between PKCS#1v1.5 and PSS for RSA keys), mostly for the sake of X.509. This information is now represented as `mbedtls_pk_sigalg_t`, using enum constants with tweaked names:

    | Old `mbedtls_pk_type_t` name | New `mbedtls_pk_sigalg_t` name |
    | ---------------------------- | ------------------------------ |
    | `MBEDTLS_PK_NONE` | `MBEDTLS_PK_SIGALG_NONE` |
    | `MBEDTLS_PK_RSA` | `MBEDTLS_PK_SIGALG_RSA_PKCS1V15` |
    | `MBEDTLS_PK_RSASSA_PSS` | `MBEDTLS_PK_SIGALG_RSA_PSS` |
    | `MBEDTLS_PK_ECDSA` or `MBEDTLS_PK_ECKEY` | `MBEDTLS_PK_SIGALG_ECDSA` |

* Policy information after parsing a key (only relevant for ECC keys marked as ECDH-only). This is now tracked internally and reflected in rejecting the key for signature or verification in `mbedtls_pk_get_psa_attributes()` and in operation functions.
  <!-- Untested, see https://github.com/Mbed-TLS/TF-PSA-Crypto/issues/206 -->

As a consequence, the functions `mbedtls_pk_get_type()`, `mbedtls_pk_get_name()` and `mbedtls_pk_info_from_type()` and have been removed. The type `mbedtls_pk_info_t` is no longer part of the API.

The function `mbedtls_pk_get_len()` has also been removed. It was not very meaningful since it did not convey the length of the key representation, but the size in bytes of the representation of one number associated with the key. As before, you can use `mbedtls_pk_get_bitlen()` to get the key size in the usual cryptographic sense. The size of a formatted key representation depends on the format, and there is no API function to determine it. (For the short export formats of PSA, you can use macros such as `PSA_EXPORT_KEY_OUTPUT_SIZE()`, `PSA_EXPORT_PUBLIC_KEY_OUTPUT_SIZE()`, `PSA_EXPORT_KEY_PAIR_MAX_SIZE`, `PSA_EXPORT_PUBLIC_KEY_MAX_SIZE`.)

### Checking the capabilities of a PK context

The function `mbedtls_pk_can_do()`, based on the polysemic `mbedtls_pk_type_t`, has been removed. The function `mbedtls_pk_can_do_ext()`, based on PSA metadata, has been renamed to `mbedtls_pk_can_do_psa()`. It now supports public-key operations as well as private-key operations.

### Loss of policy information for RSA keys

The PK module no longer partially keeps track of whether an RSA key is intended for use with PKCS#1v1.5 or PKCS#1v2.1 (PSS or OAEP) algorithms. Functions that consume a PK object (`mbedtls_pk_get_psa_attributes()`, `mbedtls_pk_sign()`, `mbedtls_pk_verify()`) now always default to PKCS#1v1.5, except for PK contexts populated with `mbedtls_pk_wrap_psa()` where they use the primary algorithm in the key's policy.

Note in particular that the functions `mbedtls_pk_copy_from_psa()` and `mbedtls_pk_copy_public_from_psa()` are now equivalent to exporting and re-importing the key, losing all policy information. For example, calling `mbedtls_pk_copy_from_psa()` on a key whose policy specifies PSS as the single allowed algorithm, then calling `mbedtls_pk_sign()`, results in a PKCS#1v1.5 signature. Call `mbedtls_pk_sign_ext()` or `mbedtls_pk_verify_ext()` if you want to specify the signature algorithm explicitly.

### Removal of transparent PK contexts

It is no longer possible to directly inspect a PK context to act on its underlying RSA or ECC context. The representation of a key in PK is now always an implementation detail.

As a consequence, the functions `mbedtls_pk_setup()`, `mbedtls_pk_rsa()` and `mbedtls_pk_ec()` have been removed.

### Changes to opaque PK contexts

The library no longer supports an alternative representation of RSA keys in PK (`MBEDTLS_PK_RSA_ALT_SUPPORT` compilation option, PK context type `MBEDTLS_PK_RSA_ALT`). See [“RSA-ALT interface” in the PSA transition guide](psa-transition.md#rsa-alt-interface).

A PK object can still wrap around an PSA key, which was formerly known as “opaque” PK objects of type `MBEDTLS_PK_OPAQUE`. The function `mbedtls_pk_setup_opaque()` has been renamed to `mbedtls_pk_wrap_psa()`, to reflect the fact that the PSA key is not necessarily “opaque” in PSA terminology. The PK module now hides the distinction between “wrapping” and “non-wrapping” PK contexts as much as possible, and in particular there is no simple metadata query to distinguish “wrapping” keys like the former `mbedtls_pk_get_type()`.

### Fewer operations in PK

The functions `mbedtls_pk_encrypt()` and `mbedtls_pk_decrypt()` have been removed. Use `psa_asymmetric_encrypt()` and `psa_asymmetric_decrypt()` instead. See [“Asymmetric encryption and decryption” in the PSA transition guide](psa-transition.md#rsa-alt-interface) for more information.

If you have a key as a PK context and you wish to use it with PSA operation functions, use the functions `mbedtls_pk_get_psa_attributes()` and `mbedtls_pk_import_into_psa()` to obtain a PSA key identifier. See the documentation of these functions or [“Creating a PSA key via PK” in the PSA transition guide](psa-transition.md#creating-a-psa-key-via-pk) for more information.

### Miscellaneous changes to PK signature and verification

The `type` argument to `mbedtls_pk_sign_ext()` and `mbedtls_pk_verify_ext()` now has the type `mbedtls_pk_sigalg_t`. See [“Changes to PK metadata”](#changes-to-pk-metadata).

The function `mbedtls_pk_verify_ext()` no longer supports a custom salt length for RSA-PSS. It always allows signatures with any salt length. The PSA API offers `psa_verify_hash()` with `PSA_ALG_RSA_PSS` to enforce the most commonly used salt length.

`MBEDTLS_ERR_PK_SIG_LEN_MISMATCH` is no longer a distinct error code. A valid signature with trailing garbage is now reported as an invalid signature with all algorithms.

The PK module continues to favor deterministic ECDSA over randomized ECDSA when both are possible. This may change in the future. The new macro `MBEDTLS_PK_ALG_ECDSA(hash_alg)` indicates which variant PK favors. It is equivalent to either `PSA_ALG_DETERMINISTIC_ECDSA(hash_alg)` or `PSA_ALG_ECDSA(hash_alg)`.

### Changes in resources consumed by PK objects

The implementation of the type `mbedtls_pk_context` has changed in ways that do not directly affect functionality, but affect resource consumption and memory partitioning.

Historically, at least in the default configuration, a PK context stored all the key material in a heap-allocated object. In TF-PSA-Crypto 1.0, a PK context now contains:

* the public key, in a fixed-size buffer that is directly in the `mbedtls_pk_context` structure;
* the private key (if available), in the PSA key store.

This may change in future minor versions of TF-PSA-Crypto.

### Other miscellaneous changes in PK

Since PSA has a built-in random generator, all `(f_rng, p_rng)` arguments to PK functions have been removed.

`mbedtls_pk_debug()` and the associated types have been removed. This was intended solely for internal consumption by the debug module, and tied to internal details of the legacy representation of keys. If you need equivalent functionality in TF-PSA-Crypto, export the key.

The auxiliary functions `mbedtls_pk_parse_subpubkey()` and `mbedtls_pk_write_pubkey()` are no longer exposed. Use `mbedtls_pk_parse_public_key()` and `mbedtls_pk_write_pubkey_der()` instead.
