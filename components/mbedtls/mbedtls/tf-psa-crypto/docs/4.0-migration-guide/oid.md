## OID module

The compilation option `MBEDTLS_OID_C` no longer exists. OID tables are included in the build automatically as needed for parsing and writing keys and signatures.

TF-PSA-Crypto does not have interfaces to look up values by OID or OID by enum values.

Functions to convert between binary and dotted string OID representations (`mbedtls_oid_get_numeric_string()` and `mbedtls_oid_from_numeric_string()`) are still available, but they are now in the X.509 library in Mbed TLS. The header file `<mbedtls/oid.h>` is now in Mbed TLS, not in TF-PSA-Crypto.

TF-PSA-Crypto does not expose OID values through macros, the way Mbed TLS 3.x and earlier did.
