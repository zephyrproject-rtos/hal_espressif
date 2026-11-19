## Error codes

### Unified error code space

The convention still applies that functions return 0 for success and a negative value between -32767 and -1 on error. PSA functions (`psa_xxx()` or `mbedtls_psa_xxx()`) still return a `PSA_ERROR_xxx` error codes. Non-PSA functions (`mbedtls_xxx()` excluding `mbedtls_psa_xxx()`) can return either `PSA_ERROR_xxx` or `MBEDTLS_ERR_xxx` error codes.

There may be cases where an `MBEDTLS_ERR_xxx` constant has the same numerical value as a `PSA_ERROR_xxx`. In such cases, they have the same meaning: they are different names for the same error condition.

### Simplified legacy error codes

All values returned by a function to indicate an error now have a defined constant named `MBEDTLS_ERR_xxx` or `PSA_ERROR_xxx`. Functions no longer return the sum of a “low-level” and a “high-level” error code.

Generally, functions that used to return the sum of two error codes now return the low-level code. However, as before, the exact error code returned in a given scenario can change without notice unless the condition is specifically described in the function's documentation and no other condition is applicable.

As a consequence, the functions `mbedtls_low_level_strerr()` and `mbedtls_high_level_strerr()` no longer exist.

### Removed English error messages

TF-PSA-Crypto does not provide English text corresponding to error codes. The functionality provided by `mbedtls_strerror()` in `mbedtls/error.h` is still present in Mbed TLS.

### Removed error code names

Many legacy error codes have been removed in favor of PSA error codes. Generally, functions that returned a legacy error code in the table below in Mbed TLS 3.6 now return the PSA error code listed on the same row. Similarly, callbacks should apply the same changes to error code, unless there has been a relevant change to the callback's interface.

| Legacy constant (Mbed TLS 3.6)             | PSA constant (TF-PSA-Crypto 1.0) |
|--------------------------------------------|----------------------------------|
| `MBEDTLS_ERR_AES_BAD_INPUT_DATA`           | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_ARIA_BAD_INPUT_DATA`          | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_ASN1_ALLOC_FAILED`            | `PSA_ERROR_INSUFFICIENT_MEMORY`  |
| `MBEDTLS_ERR_ASN1_BUF_TOO_SMALL`           | `PSA_ERROR_BUFFER_TOO_SMALL`     |
| `MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL`      | `PSA_ERROR_BUFFER_TOO_SMALL`     |
| `MBEDTLS_ERR_CAMELLIA_BAD_INPUT_DATA`      | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_CCM_AUTH_FAILED`              | `PSA_ERROR_INVALID_SIGNATURE`    |
| `MBEDTLS_ERR_CCM_BAD_INPUT_DATA`           | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_CHACHA20_BAD_INPUT_DATA`      | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_CHACHAPOLY_AUTH_FAILED`       | `PSA_ERROR_INVALID_SIGNATURE`    |
| `MBEDTLS_ERR_CIPHER_ALLOC_FAILED`          | `PSA_ERROR_INSUFFICIENT_MEMORY`  |
| `MBEDTLS_ERR_CIPHER_AUTH_FAILED`           | `PSA_ERROR_INVALID_SIGNATURE`    |
| `MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA`        | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_CIPHER_INVALID_PADDING`       | `PSA_ERROR_INVALID_PADDING`      |
| `MBEDTLS_ERR_ECP_ALLOC_FAILED`             | `PSA_ERROR_INSUFFICIENT_MEMORY`  |
| `MBEDTLS_ERR_ECP_BAD_INPUT_DATA`           | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_ECP_BUFFER_TOO_SMALL`         | `PSA_ERROR_BUFFER_TOO_SMALL`     |
| `MBEDTLS_ERR_ECP_IN_PROGRESS`              | `PSA_OPERATION_INCOMPLETE`       |
| `MBEDTLS_ERR_ECP_SIG_LEN_MISMATCH`         | `PSA_ERROR_INVALID_SIGNATURE`    |
| `MBEDTLS_ERR_ECP_VERIFY_FAILED`            | `PSA_ERROR_INVALID_SIGNATURE`    |
| `MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED`    | `PSA_ERROR_CORRUPTION_DETECTED`  |
| `MBEDTLS_ERR_ERROR_GENERIC_ERROR`          | `PSA_ERROR_GENERIC_ERROR`        |
| `MBEDTLS_ERR_GCM_AUTH_FAILED`              | `PSA_ERROR_INVALID_SIGNATURE`    |
| `MBEDTLS_ERR_GCM_BAD_INPUT`                | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_GCM_BUFFER_TOO_SMALL`         | `PSA_ERROR_BUFFER_TOO_SMALL`     |
| `MBEDTLS_ERR_LMS_ALLOC_FAILED`             | `PSA_ERROR_INSUFFICIENT_MEMORY`  |
| `MBEDTLS_ERR_LMS_BAD_INPUT_DATA`           | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_LMS_BUFFER_TOO_SMALL`         | `PSA_ERROR_BUFFER_TOO_SMALL`     |
| `MBEDTLS_ERR_MD_ALLOC_FAILED`              | `PSA_ERROR_INSUFFICIENT_MEMORY`  |
| `MBEDTLS_ERR_MD_BAD_INPUT_DATA`            | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_MPI_ALLOC_FAILED`             | `PSA_ERROR_INSUFFICIENT_MEMORY`  |
| `MBEDTLS_ERR_MPI_BAD_INPUT_DATA`           | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_MPI_BUFFER_TOO_SMALL`         | `PSA_ERROR_BUFFER_TOO_SMALL`     |
| `MBEDTLS_ERR_OID_NOT_FOUND`                | `PSA_ERROR_NOT_SUPPORTED`        |
| `MBEDTLS_ERR_PEM_ALLOC_FAILED`             | `PSA_ERROR_INSUFFICIENT_MEMORY`  |
| `MBEDTLS_ERR_PEM_BAD_INPUT_DATA`           | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_PK_ALLOC_FAILED`              | `PSA_ERROR_INSUFFICIENT_MEMORY`  |
| `MBEDTLS_ERR_PK_BAD_INPUT_DATA`            | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_PK_BUFFER_TOO_SMALL`          | `PSA_ERROR_BUFFER_TOO_SMALL`     |
| `MBEDTLS_ERR_PK_SIG_LEN_MISMATCH`          | `PSA_ERROR_INVALID_SIGNATURE`    |
| `MBEDTLS_ERR_PKCS5_BAD_INPUT_DATA`         | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_PLATFORM_FEATURE_UNSUPPORTED` | `PSA_ERROR_NOT_SUPPORTED`        |
| `MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED`     | `PSA_ERROR_HARDWARE_FAILURE`     |
| `MBEDTLS_ERR_POLY1305_BAD_INPUT_DATA`      | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_RSA_BAD_INPUT_DATA`           | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_RSA_INVALID_PADDING`          | `PSA_ERROR_INVALID_PADDING`      |
| `MBEDTLS_ERR_RSA_OUTPUT_TOO_LARGE`         | `PSA_ERROR_BUFFER_TOO_SMALL`     |
| `MBEDTLS_ERR_SHA1_BAD_INPUT_DATA`          | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_SHA256_BAD_INPUT_DATA`        | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_SHA3_BAD_INPUT_DATA`          | `PSA_ERROR_INVALID_ARGUMENT`     |
| `MBEDTLS_ERR_SHA512_BAD_INPUT_DATA`        | `PSA_ERROR_INVALID_ARGUMENT`     |
