## Function prototype changes

A number of existing functions now take a different list of arguments, mostly to migrate them to the PSA API.

### Public functions no longer take a RNG callback

Functions that need randomness no longer take an RNG callback in the form of `f_rng, p_rng` arguments. Instead, they use the PSA Crypto random generator (accessible as `psa_generate_random()`). All software using the LMS or PK modules must call `psa_crypto_init()` before calling any of the functions listed here.

### RNG removal in LMS

The following function prototypes have been changed in `mbedtls/lms.h`:

```c
int mbedtls_lms_generate_private_key(mbedtls_lms_private_t *ctx, mbedtls_lms_algorithm_type_t type, mbedtls_lmots_algorithm_type_t otstype,
                                     int (*f_rng)(void *, unsigned char *, size_t),
                                     void *p_rng,
                                     const unsigned char *seed, size_t seed_size);

int mbedtls_lms_sign(mbedtls_lms_private_t *ctx,
                     int (*f_rng)(void *, unsigned char *, size_t),
                     void *p_rng,
                     const unsigned char *msg, unsigned int msg_size, unsigned char *sig, size_t sig_size, size_t *sig_len);
```


to

```c
int mbedtls_lms_generate_private_key(mbedtls_lms_private_t *ctx, mbedtls_lms_algorithm_type_t type, mbedtls_lmots_algorithm_type_t otstype,
                                     const unsigned char *seed, size_t seed_size);

int mbedtls_lms_sign(mbedtls_lms_private_t *ctx,
                     const unsigned char *msg, unsigned int msg_size, unsigned char *sig, size_t sig_size, size_t *sig_len);
```

### RNG removal in PK

The following function prototypes have been changed in `mbedtls/pk.h`:

```c
int mbedtls_pk_sign_restartable(mbedtls_pk_context *ctx, mbedtls_md_type_t md_alg, const unsigned char *hash, size_t hash_len, unsigned char *sig, size_t sig_size, size_t *sig_len,
                                int (*f_rng)(void *, unsigned char *, size_t),
                                void *p_rng,
                                mbedtls_pk_restart_ctx *rs_ctx);

int mbedtls_pk_check_pair(const mbedtls_pk_context *pub, const mbedtls_pk_context *prv,
                          int (*f_rng)(void *, unsigned char *, size_t),
                          void *p_rng);

int mbedtls_pk_sign(mbedtls_pk_context *ctx, mbedtls_md_type_t md_alg, const unsigned char *hash, size_t hash_len, unsigned char *sig, size_t sig_size, size_t *sig_len,
                    int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);

int mbedtls_pk_sign_ext(mbedtls_pk_type_t pk_type, mbedtls_pk_context *ctx, mbedtls_md_type_t md_alg, const unsigned char *hash, size_t hash_len, unsigned char *sig, size_t sig_size, size_t *sig_len,
                        int (*f_rng)(void *, unsigned char *, size_t),
                        void *p_rng);

int mbedtls_pk_parse_key(mbedtls_pk_context *ctx, const unsigned char *key, size_t keylen, const unsigned char *pwd, size_t pwdlen,
                         int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);

int mbedtls_pk_parse_keyfile(mbedtls_pk_context *ctx, const char *path, const char *password,
                             int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);

typedef int (*mbedtls_pk_rsa_alt_sign_func)(void *ctx,
                                            int (*f_rng)(void *, unsigned char *, size_t),
                                            void *p_rng,
                                            mbedtls_md_type_t md_alg, unsigned int hashlen, const unsigned char *hash, unsigned char *sig);
```

to

```c
int mbedtls_pk_sign_restartable(mbedtls_pk_context *ctx, mbedtls_md_type_t md_alg, const unsigned char *hash, size_t hash_len, unsigned char *sig, size_t sig_size, size_t *sig_len,
                                 mbedtls_pk_restart_ctx *rs_ctx);

int mbedtls_pk_check_pair(const mbedtls_pk_context *pub, const mbedtls_pk_context *prv);

int mbedtls_pk_sign(mbedtls_pk_context *ctx, mbedtls_md_type_t md_alg, const unsigned char *hash, size_t hash_len, unsigned char *sig, size_t sig_size, size_t *sig_len);

int mbedtls_pk_sign_ext(mbedtls_pk_type_t pk_type, mbedtls_pk_context *ctx, mbedtls_md_type_t md_alg, const unsigned char *hash, size_t hash_len, unsigned char *sig, size_t sig_size, size_t *sig_len);

int mbedtls_pk_parse_key(mbedtls_pk_context *ctx, const unsigned char *key, size_t keylen, const unsigned char *pwd, size_t pwdlen);

int mbedtls_pk_parse_keyfile(mbedtls_pk_context *ctx, const char *path, const char *password);

typedef int (*mbedtls_pk_rsa_alt_sign_func)(void *ctx,
                                            mbedtls_md_type_t md_alg, unsigned int hashlen, const unsigned char *hash, unsigned char *sig);
```

### Changes to NIST_KW

The NIST\_KW module remains part of TF-PSA-Crypto, since it does not have a PSA equivalent yet. However, its interface has changed, to use PSA key identifiers instead of a plaintext key via a custom context.

The following function prototypes have been changed in `mbedtls/nist_kw.h`:

```c
int mbedtls_nist_kw_wrap(mbedtls_nist_kw_context *ctx,
                         mbedtls_nist_kw_mode_t mode,
                         const unsigned char *input, size_t in_len,
                         unsigned char *output, size_t *out_len, size_t out_size);
int mbedtls_nist_kw_unwrap(mbedtls_nist_kw_context *ctx,
                           mbedtls_nist_kw_mode_t mode,
                           const unsigned char *input, size_t in_len,
                           unsigned char *output, size_t *out_len, size_t out_size);
```

to

```c
psa_status_t mbedtls_nist_kw_wrap(mbedtls_svc_key_id_t key,
                                  mbedtls_nist_kw_mode_t mode,
                                  const unsigned char *input, size_t input_length,
                                  unsigned char *output, size_t output_size,
                                  size_t *output_length);
psa_status_t mbedtls_nist_kw_unwrap(mbedtls_svc_key_id_t key,
                                    mbedtls_nist_kw_mode_t mode,
                                    const unsigned char *input, size_t input_length,
                                    unsigned char *output, size_t output_size,
                                    size_t *output_length);
```

Note that in addition to the change to keys, the last two parameters for the output buffer size and the length of the actual output have been swapped, to align the order of parameters with PSA Crypto APIs.

The type `mbedtls_nist_kw_context` and the functions `mbedtls_nist_kw_init()`, `mbedtls_nist_kw_free()` and `mbedtls_nist_kw_setkey()` have been removed, since the interface no longer involves a specific context object for NIST\_KW.

Also, the function `mbedtls_nist_kw_self_test()` has been removed. TF-PSA-Crypto 1.0 does not provide a ready-made self-test interface; one may be added in a future version of the library.

### Changes to ASN.1 functions

As a consequence of the removal of the type `mbedtls_mpi` (provided by `mbedtls/bignum.h`) from public interfaces, the ASN.1 functions to parse and write integers have changed.

The following functions have been removed from the API:

```c
// mbedtls/asn1.h
int mbedtls_asn1_get_mpi(unsigned char **p, const unsigned char *end,
                         mbedtls_mpi *X);

// mbedtls/asn1write.h
int mbedtls_asn1_write_mpi(unsigned char **p, const unsigned char *start,
                           const mbedtls_mpi *X);
```

You can use the following new functions instead:

```c
// mbedtls/asn1.h
int mbedtls_asn1_get_integer(unsigned char **p, const unsigned char *end,
                             unsigned char **head, size_t *length);

// mbedtls/asn1write.h
int mbedtls_asn1_write_integer(unsigned char **p,
                               unsigned char *start,
                               const unsigned char *integer,
                               size_t integer_length);
```

Both new functions use a big-endian byte buffer as the representation.

Note some differences in the semantics of the new parsing function:

* `mbedtls_asn1_get_integer()` rejects negative integers. (`mbedtls_asn1_get_mpi()` misparsed them, using the sign bit as a value bit).
* `mbedtls_asn1_get_integer()` does not allocate memory. It returns a pointer inside the ASN.1 representation.
