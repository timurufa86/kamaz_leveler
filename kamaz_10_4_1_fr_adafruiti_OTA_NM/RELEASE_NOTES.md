# V 10.4.1

- Fix OTA «нет связи TLS»: root cause `SSL - Memory allocation failed` (maxBlk ~34K < 2×16K mbedtls)
- Disable CA cert bundle for GitHub HTTPS; integrity remains SHA-256 of `kamaz_leveler.bin`
- Heap defrag before TLS; pause ButtonTask during OTA TLS
- Reduce Button/Display/OTA task stacks to free contiguous DRAM for TLS
- Lower maxBlk wait thresholds; log mbedtls lastError on GET fail
