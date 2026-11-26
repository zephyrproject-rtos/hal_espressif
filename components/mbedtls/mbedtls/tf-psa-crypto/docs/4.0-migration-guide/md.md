## Reduced `md.h`

PSA is now the preferred interface for computing hashes and HMAC. See [“Hashes and MAC” in the PSA transition guide](psa-transition.md#hashes-and-mac) for more information.

TF-PSA-Crypto 1.x still has a header file `<mbedtls/md.h>` to facilitate the transition. Its functionality is limited to calculating hashes:

* `mbedtls_md_setup()` now requires the `hmac` parameter to be 0. Use the PSA API for HMAC calculations.
* The HMAC functions `mbedtls_md_hmac_xxx()` are no longer available.
* The metadata functions `mbedtls_md_list()`, `mbedtls_md_info_from_string()`, `mbedtls_md_get_name()` and `mbedtls_md_info_from_ctx()` have been removed. The library does not associate names to individual algorithms any longer.
* The function `mbedtls_md_file()` has been removed. To hash a file, load it into memory manually. Load it piecewise and call `mbedtls_md_update()` or `psa_hash_update()` in a loop if the file may be large.
