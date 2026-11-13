#!/usr/bin/env python3
#
# Copyright The Zephyr Project Contributors
#
# SPDX-License-Identifier: Apache-2.0

"""Build the PSA Crypto dispatch for the ESP32 SHA driver.

TF-PSA-Crypto's driver-wrapper generator does not emit hash entry points (they
are hand-written in its template), so the dispatch used here is the stock
TF-PSA-Crypto ``dispatch/`` directory with the driver's branches inserted into
the six ``psa_driver_wrapper_hash_*`` functions, its context in the hash
union and its ID in the driver enumeration.

Usage: gen_dispatch.py <tf-psa-crypto>/dispatch <output-dir>

Every insertion point must be found exactly once; anything else is an error,
so a TF-PSA-Crypto update that moves the code fails here instead of producing
a dispatch that silently bypasses the driver.
"""

import shutil
import sys
from pathlib import Path

GUARD = "ESP_PSA_SHA_DRIVER_ENABLED"
TAG = "/* ESP32 SHA driver (hal_espressif zephyr/common/psa_crypto) */"
ACCEL_END = "#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */"

WRAPPERS = "psa_crypto_driver_wrappers.h"
CONTEXTS = "include/psa/crypto_driver_contexts_primitives.h"


def die(msg):
    sys.exit(f"gen_dispatch: {msg}")


def replace_once(text, old, new, what):
    count = text.count(old)
    if count != 1:
        die(f"{what}: anchor found {count} times, expected 1")
    return text.replace(old, new)


def function_span(text, name):
    start = text.find(f"static inline psa_status_t {name}(")
    if start < 0 or text.count(f"static inline psa_status_t {name}(") != 1:
        die(f"{name}: function not found exactly once")
    end = text.find("\n}\n", start)
    if end < 0:
        die(f"{name}: end of function not found")
    return start, end


def insert_before_accel_end(text, name, block):
    start, end = function_span(text, name)
    body = text[start:end]
    if body.count(ACCEL_END) != 1:
        die(f"{name}: accelerator block end found {body.count(ACCEL_END)} times, expected 1")
    body = body.replace(ACCEL_END, block + ACCEL_END)
    return text[:start] + body + text[end:]


def edit_wrappers(text):
    text = replace_once(
        text,
        "/* END-driver headers */",
        f"""{TAG}
#if defined({GUARD})
#ifndef PSA_CRYPTO_DRIVER_PRESENT
#define PSA_CRYPTO_DRIVER_PRESENT
#endif
#ifndef PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT
#define PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT
#endif
#include "esp_psa_sha.h"
#endif /* {GUARD} */

/* END-driver headers */""",
        "driver headers",
    )

    start = text.find("/* BEGIN-driver id definition */")
    end = text.find("};", start)
    if start < 0 or end < 0:
        die("driver id enumeration not found")
    text = (text[:end] + f"""#if defined({GUARD})
    ESP_PSA_SHA_DRIVER_ID,
#endif /* {GUARD} */
""" + text[end:])

    text = insert_before_accel_end(text, "psa_driver_wrapper_hash_compute", f"""{TAG}
#if defined({GUARD})
    status = esp_psa_sha_hash_compute( alg, input, input_length,
                                       hash, hash_size, hash_length );
    if( status != PSA_ERROR_NOT_SUPPORTED )
        return( status );
#endif /* {GUARD} */
""")
    text = insert_before_accel_end(text, "psa_driver_wrapper_hash_setup", f"""{TAG}
#if defined({GUARD})
    status = esp_psa_sha_hash_setup( &operation->ctx.esp_psa_sha_ctx, alg );
    if( status == PSA_SUCCESS )
        operation->id = ESP_PSA_SHA_DRIVER_ID;

    if( status != PSA_ERROR_NOT_SUPPORTED )
        return( status );
#endif /* {GUARD} */
""")
    text = insert_before_accel_end(text, "psa_driver_wrapper_hash_clone", f"""{TAG}
#if defined({GUARD})
        case ESP_PSA_SHA_DRIVER_ID:
            target_operation->id = ESP_PSA_SHA_DRIVER_ID;
            return( esp_psa_sha_hash_clone(
                        &source_operation->ctx.esp_psa_sha_ctx,
                        &target_operation->ctx.esp_psa_sha_ctx ) );
#endif /* {GUARD} */
""")
    text = insert_before_accel_end(text, "psa_driver_wrapper_hash_update", f"""{TAG}
#if defined({GUARD})
        case ESP_PSA_SHA_DRIVER_ID:
            return( esp_psa_sha_hash_update(
                        &operation->ctx.esp_psa_sha_ctx,
                        input, input_length ) );
#endif /* {GUARD} */
""")
    text = insert_before_accel_end(text, "psa_driver_wrapper_hash_finish", f"""{TAG}
#if defined({GUARD})
        case ESP_PSA_SHA_DRIVER_ID:
            return( esp_psa_sha_hash_finish(
                        &operation->ctx.esp_psa_sha_ctx,
                        hash, hash_size, hash_length ) );
#endif /* {GUARD} */
""")
    text = insert_before_accel_end(text, "psa_driver_wrapper_hash_abort", f"""{TAG}
#if defined({GUARD})
        case ESP_PSA_SHA_DRIVER_ID:
            return( esp_psa_sha_hash_abort(
                        &operation->ctx.esp_psa_sha_ctx ) );
#endif /* {GUARD} */
""")
    return text


def edit_contexts(text):
    cc3xx = """#if defined(PSA_CRYPTO_DRIVER_CC3XX)
#include "cc3xx_crypto_primitives_private.h"
#endif /* PSA_CRYPTO_DRIVER_CC3XX */"""
    text = replace_once(text, cc3xx, cc3xx + f"""

{TAG}
#if defined({GUARD})
#include "esp_psa_sha_ctx.h"
#endif /* {GUARD} */""", "context headers")
    text = replace_once(text, "} psa_driver_hash_context_t;", f"""#if defined({GUARD})
    esp_psa_sha_hash_operation_t esp_psa_sha_ctx;
#endif /* {GUARD} */
}} psa_driver_hash_context_t;""", "hash context union")
    return text


def main():
    if len(sys.argv) != 3:
        die("usage: gen_dispatch.py <tf-psa-crypto>/dispatch <output-dir>")
    src = Path(sys.argv[1])
    out = Path(sys.argv[2])
    for rel in (WRAPPERS, CONTEXTS):
        if not (src / rel).is_file():
            die(f"{src / rel}: not found")
    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(src, out)
    (out / WRAPPERS).write_text(edit_wrappers((src / WRAPPERS).read_text()))
    (out / CONTEXTS).write_text(edit_contexts((src / CONTEXTS).read_text()))


if __name__ == "__main__":
    main()
