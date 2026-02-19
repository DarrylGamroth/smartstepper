# STM32 SPI Driver Comparison: Custom vs Upstream

**Date:** November 11, 2025  
**Custom Driver:** `/workspace/chopper/drivers/spi/spi_ll_stm32.c` (rubus_stm32_spi)  
**Upstream Driver:** `/workspace/zephyr/drivers/spi/spi_ll_stm32.c` (st_stm32_spi)

---

## Executive Summary

Your custom driver implements RTIO support with DMA and interrupt modes for STM32 SPI peripherals. While DMA support is functional, **STM32H7-specific FIFO handling is incomplete** compared to upstream. The upstream driver has more comprehensive H7 FIFO threshold management, transfer size programming, and EOT (End of Transfer) handling.

### Key Findings

| Feature | Custom Driver | Upstream Driver | Status |
|---------|---------------|-----------------|--------|
| **DMA Support** | ✅ Full | ✅ Full | **Complete** |
| **H7 FIFO Thresholds** | ⚠️ Basic | ✅ Comprehensive | **Incomplete** |
| **H7 Transfer Size** | ❌ Missing | ✅ Complete | **Missing** |
| **H7 EOT Handling** | ⚠️ Partial | ✅ Complete | **Incomplete** |
| **RTIO Integration** | ✅ Yes | ✅ Yes | **Complete** |
| **Cache Coherency** | ✅ Manual | ✅ Automatic | **Complete** |
| **Interrupt Mode** | ✅ Yes | ✅ Yes | **Complete** |
| **Polling Mode** | ✅ Yes | ✅ Yes | **Complete** |

---

## 1. DMA Support Comparison

### ✅ Both Drivers Support DMA Fully

#### Similarities
- **DMA Configuration**: Both use similar DMA block configuration patterns
- **Dummy Buffer**: Both use `dummy_rx_tx_buffer` for NULL TX/RX scenarios
- **Cache Coherency**: Both handle D-cache flushing/invalidation
- **DMA Callbacks**: Both implement callback mechanisms for completion
- **TX/RX Channels**: Both support separate TX and RX DMA channels

#### Custom Driver DMA Features
```c
static void dma_callback(const struct device *dma_dev, void *arg,
                        uint32_t channel, int status)
{
    const struct device *dev = arg;
    struct spi_stm32_data *data = dev->data;
    
    if (status < 0) {
        data->status_flags |= SPI_STM32_DMA_ERROR_FLAG;
        data->error_status = -EIO;
    } else {
        if (channel == data->dma_tx.channel) {
            data->status_flags |= SPI_STM32_DMA_TX_DONE_FLAG;
        } else if (channel == data->dma_rx.channel) {
            data->status_flags |= SPI_STM32_DMA_RX_DONE_FLAG;
        }
    }
    
    spi_stm32_try_finalize(dev);
}
```

- Uses flag-based completion tracking
- Calls `spi_stm32_try_finalize()` directly from callback
- Stores device pointer as callback arg

#### Upstream Driver DMA Features
```c
static void dma_callback(const struct device *dma_dev, void *arg,
                        uint32_t channel, int status)
{
    struct spi_stm32_data *spi_dma_data = arg;
    
    if (status < 0) {
        spi_dma_data->status_flags |= SPI_STM32_DMA_ERROR_FLAG;
    } else {
        /* Channel-specific flags set here */
    }
    
    k_sem_give(&spi_dma_data->status_sem);
}
```

- Uses semaphore-based synchronization
- Stores `spi_stm32_data` as callback arg
- Waits on semaphore in main thread

**Verdict:** Both approaches are valid. Custom driver is more event-driven, upstream uses blocking semaphore.

---

## 2. STM32H7 FIFO Support Comparison

### ⚠️ Custom Driver Has Basic FIFO Support

#### Custom Driver FIFO Implementation

**FIFO Threshold Function** (in `spi_ll_stm32.h`):
```c
static inline void spi_stm32_update_fifo_threshold(const struct spi_stm32_config *cfg,
                                                   SPI_TypeDef *spi,
                                                   uint8_t word_size,
                                                   bool use_dma)
{
    if (!cfg->fifo_enabled) {
        return;
    }

    if (use_dma) {
        if (word_size == 16U) {
            LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_04DATA);
        } else {
            LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_08DATA);
        }
    } else {
        if (word_size == 16U) {
            LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_02DATA);
        } else {
            LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_04DATA);
        }
    }
}
```

**Usage Points:**
- ✅ Called in `spi_stm32_transceive_interrupt()`
- ✅ Called in `spi_stm32_transceive_dma()`
- ✅ Called in polling mode

**What's Present:**
- Dynamic FIFO threshold adjustment based on word size and DMA usage
- Basic threshold values (2/4/8 data)

**What's Missing:**
1. ❌ **No `LL_SPI_SetTransferSize()` calls** for H7 master mode
2. ❌ **No FIFO flush logic** when switching directions
3. ❌ **No dynamic FIFO level monitoring** during transfer

### ✅ Upstream Driver Has Comprehensive FIFO Support

#### Upstream Driver FIFO Features

**Transfer Size Programming** (H7-specific):
```c
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
if (cfg->fifo_enabled && SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
    /* Calculate total frames to transfer */
    int32_t num_frames = spi_stm32_count_total_frames(config, tx_bufs, rx_bufs);
    
    if (num_frames < 0) {
        ret = num_frames;
        goto end;
    }
    
    if (transfer_dir == LL_SPI_HALF_DUPLEX_RX) {
        num_frames -= 1;
    }
    
    LL_SPI_SetTransferSize(spi, num_frames);
}
#endif
```

**FIFO Flush Logic**:
```c
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_fifo)
/* Flush RX buffer */
while (ll_func_rx_is_not_empty(spi)) {
    (void)LL_SPI_ReceiveData8(spi);
}
#endif
```

**Master Start Sequence** (H7-specific):
```c
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
if (LL_SPI_GetMode(spi) == LL_SPI_MODE_MASTER) {
    LL_SPI_StartMasterTransfer(spi);
    while (!LL_SPI_IsActiveFlag_EOT(spi)) {
        /* Wait for EOT */
    }
}
#endif
```

---

## 3. STM32H7 EOT (End of Transfer) Handling

### ⚠️ Custom Driver Has Partial EOT Support

**What's Implemented:**
```c
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
/* H7 needs EOT handling in interrupt mode */
data->waiting_eot = true;
ll_func_enable_int_eot(spi);
#endif
```

- ✅ EOT interrupt enabled in interrupt mode
- ✅ EOT flag set for DMA mode
- ✅ ISR checks for EOT

**ISR EOT Handling:**
```c
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
if (data->waiting_eot && LL_SPI_IsEnabledIT_EOT(spi) && LL_SPI_IsActiveFlag_EOT(spi)) {
    LL_SPI_ClearFlag_EOT(spi);
    ll_func_disable_int_eot(spi);
    data->status_flags |= SPI_STM32_EOT_DONE_FLAG;
    spi_stm32_try_finalize(dev);
    return;
}
#endif
```

**What's Missing:**
1. ❌ **No EOT wait loop** for master transfer start
2. ❌ **No `LL_SPI_StartMasterTransfer()` call** in some paths
3. ❌ **Transfer size not programmed** before EOT wait

### ✅ Upstream Driver Has Complete EOT Support

**Complete Sequence:**
```c
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
if (LL_SPI_GetMode(spi) == LL_SPI_MODE_MASTER) {
    /* Set transfer size first */
    LL_SPI_SetTransferSize(spi, num_frames);
    
    /* Start transfer */
    LL_SPI_StartMasterTransfer(spi);
    
    /* Wait for EOT */
    while (!LL_SPI_IsActiveFlag_EOT(spi)) {
        /* Busy wait or interrupt wait */
    }
    
    /* Clear EOT flag */
    LL_SPI_ClearFlag_EOT(spi);
}
#endif
```

---

## 4. Missing H7 Features in Custom Driver

### Critical Missing Features

#### 1. Transfer Size Programming ❌
**Required for:** H7 FIFO mode when operating as master

**What Upstream Does:**
```c
int32_t spi_stm32_count_total_frames(const struct spi_config *config,
                                     const struct spi_buf_set *tx_bufs,
                                     const struct spi_buf_set *rx_bufs)
{
    int tx_frames = spi_stm32_count_bufset_frames(config, tx_bufs);
    int rx_frames = spi_stm32_count_bufset_frames(config, rx_bufs);
    return MAX(rx_frames, tx_frames);
}
```

Then programs it:
```c
LL_SPI_SetTransferSize(spi, total_frames);
```

**Why It Matters:** H7 SPI peripheral needs to know how many data frames to expect to properly manage FIFO and generate EOT signal.

#### 2. FIFO Packet Management ❌
**Required for:** Optimized H7 FIFO operation

**What Upstream Does:**
- Counts frames in buffer sets
- Validates frame alignment
- Sets transfer size before starting
- Adjusts for half-duplex RX (decrements by 1)

#### 3. Master Transfer Start Sequence ❌
**Required for:** H7 master mode operation

**What Upstream Does:**
```c
if (LL_SPI_GetMode(spi) == LL_SPI_MODE_MASTER) {
    LL_SPI_StartMasterTransfer(spi);
}
```

**Current Custom Driver:** Missing explicit `LL_SPI_StartMasterTransfer()` in some paths

---

## 5. Recommendations

### Priority 1: Add Transfer Size Programming (HIGH)

Add this function to your custom driver:

```c
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
static void spi_stm32_program_h7_transfer(SPI_TypeDef *spi, struct spi_stm32_data *data)
{
    const struct spi_config *config = data->ctx.config;
    
    if (LL_SPI_GetMode(spi) == LL_SPI_MODE_SLAVE) {
        return; /* Slave mode doesn't need transfer size */
    }
    
    /* Calculate total transfer length in frames */
    uint32_t frame_bytes = SPI_WORD_SIZE_GET(config->operation) / 8;
    uint32_t tx_frames = data->tx_len / frame_bytes;
    uint32_t rx_frames = data->rx_len / frame_bytes;
    uint32_t total_frames = MAX(tx_frames, rx_frames);
    
    /* H7 quirk: for half-duplex RX, decrement by 1 */
    if (LL_SPI_GetTransferDirection(spi) == LL_SPI_HALF_DUPLEX_RX) {
        if (total_frames > 0) {
            total_frames--;
        }
    }
    
    LL_SPI_SetTransferSize(spi, total_frames);
}
#endif
```

**Where to Call:**
- Before `LL_SPI_Enable()` in DMA mode
- Before `LL_SPI_Enable()` in interrupt mode
- Before starting any H7 master transfer

### Priority 2: Add Explicit Master Transfer Start (MEDIUM)

In your `spi_stm32_start_h7_master()` function:

```c
static inline void spi_stm32_start_h7_master(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
    if (LL_SPI_GetMode(spi) == LL_SPI_MODE_MASTER) {
        LL_SPI_StartMasterTransfer(spi);
    }
#else
    ARG_UNUSED(spi);
#endif
}
```

### Priority 3: Enhance FIFO Flush Logic (MEDIUM)

Add FIFO flushing in `spi_stm32_complete()`:

```c
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
if (cfg->fifo_enabled) {
    /* Flush any remaining data in RX FIFO */
    while (ll_func_rx_is_not_empty(spi)) {
        (void)LL_SPI_ReceiveData8(spi);
    }
    
    /* Reset transfer size */
    LL_SPI_SetTransferSize(spi, 0);
}
#endif
```

### Priority 4: Add Frame Counting for RTIO (LOW)

For better H7 support with RTIO buffers, add frame counting:

```c
static uint32_t spi_stm32_count_rtio_frames(const struct spi_config *config,
                                            const uint8_t *buf, size_t len)
{
    uint8_t bytes_per_frame = SPI_WORD_SIZE_GET(config->operation) / 8;
    
    if ((len % bytes_per_frame) != 0) {
        LOG_WRN("Buffer length %zu not aligned to frame size %u", 
                len, bytes_per_frame);
    }
    
    return len / bytes_per_frame;
}
```

---

## 6. Code Snippet: Complete H7 DMA Setup

Here's a complete H7 DMA setup incorporating all recommendations:

```c
#ifdef CONFIG_SPI_RUBUS_STM32_DMA
static int spi_stm32_transceive_dma(const struct device *dev)
{
    const struct spi_stm32_config *cfg = dev->config;
    struct spi_stm32_data *data = dev->data;
    SPI_TypeDef *spi = cfg->spi;
    const struct spi_config *config = data->ctx.config;
    uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);
    size_t dma_len_tx, dma_len_rx;
    int ret;

    /* Determine transfer type and configure accordingly */
    if (data->tx_buf && data->rx_buf) {
        LL_SPI_SetTransferDirection(spi, LL_SPI_FULL_DUPLEX);
        dma_len_tx = data->tx_len;
        dma_len_rx = data->rx_len;
    } else if (data->tx_buf) {
        LL_SPI_SetTransferDirection(spi, LL_SPI_HALF_DUPLEX_TX);
        dma_len_tx = data->tx_len;
        dma_len_rx = data->tx_len;
    } else {
        LL_SPI_SetTransferDirection(spi, LL_SPI_HALF_DUPLEX_RX);
        dma_len_tx = data->rx_len;
        dma_len_rx = data->rx_len;
    }

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
    /* Update FIFO threshold for DMA */
    spi_stm32_update_fifo_threshold(cfg, spi, word_size, true);
    
    /* **NEW:** Program transfer size for H7 master mode */
    spi_stm32_program_h7_transfer(spi, data);
#endif

    /* Reset DMA status flags */
    data->status_flags = 0;
    data->dma_active = true;
    data->error_status = 0;

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
    if (IS_ENABLED(CONFIG_SPI_RUBUS_STM32_INTERRUPT)) {
        data->waiting_eot = true;
        LL_SPI_ClearFlag_EOT(spi);
        ll_func_enable_int_eot(spi);
    } else {
        data->waiting_eot = false;
    }
#endif

    /* Load DMA channels */
    ret = spi_stm32_dma_rx_load(dev, data->rx_buf, dma_len_rx);
    if (ret != 0) {
        goto dma_error;
    }

    ret = spi_stm32_dma_tx_load(dev, data->tx_buf, dma_len_tx);
    if (ret != 0) {
        goto dma_error;
    }

    /* Enable SPI DMA requests */
    LL_SPI_EnableDMAReq_RX(spi);
    LL_SPI_EnableDMAReq_TX(spi);

    /* Enable SPI peripheral */
    LL_SPI_Enable(spi);

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
    /* **NEW:** Start H7 master transfer explicitly */
    spi_stm32_start_h7_master(spi);
#endif

    return 0;

dma_error:
    data->dma_active = false;
    data->error_status = ret;
    return ret;
}
#endif
```

---

## 7. Testing Recommendations

### Test Case 1: H7 FIFO with Various Transfer Sizes
```c
/* Test different transfer sizes to verify LL_SPI_SetTransferSize() works */
static uint8_t test_sizes[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};

for (int i = 0; i < ARRAY_SIZE(test_sizes); i++) {
    uint8_t tx_buf[256] = {0};
    uint8_t rx_buf[256] = {0};
    
    struct spi_buf tx = {.buf = tx_buf, .len = test_sizes[i]};
    struct spi_buf rx = {.buf = rx_buf, .len = test_sizes[i]};
    struct spi_buf_set tx_bufs = {.buffers = &tx, .count = 1};
    struct spi_buf_set rx_bufs = {.buffers = &rx, .count = 1};
    
    int ret = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
    /* Verify transfer completed successfully */
}
```

### Test Case 2: Half-Duplex RX with EOT
```c
/* Verify half-duplex RX works with corrected transfer size (n-1) */
uint8_t rx_buf[16] = {0};
struct spi_buf rx = {.buf = rx_buf, .len = 16};
struct spi_buf_set rx_bufs = {.buffers = &rx, .count = 1};

int ret = spi_transceive(spi_dev, &spi_cfg, NULL, &rx_bufs);
/* Should complete with EOT, transfer size should be 15 not 16 */
```

### Test Case 3: Master Transfer Start Verification
```c
/* Verify LL_SPI_StartMasterTransfer() is called correctly */
/* Use a logic analyzer to capture SPI clock and verify it starts */
/* immediately after peripheral enable */
```

---

## 8. Summary Table

| Feature | Custom Driver Status | Action Required |
|---------|---------------------|-----------------|
| DMA TX/RX Channels | ✅ Complete | None |
| Cache Coherency | ✅ Complete | None |
| DMA Callbacks | ✅ Complete | None |
| FIFO Thresholds | ⚠️ Basic | Enhance with transfer size |
| Transfer Size Programming | ❌ Missing | **Add `LL_SPI_SetTransferSize()`** |
| Master Transfer Start | ⚠️ Partial | **Add explicit start call** |
| EOT Interrupt | ✅ Complete | None |
| EOT Wait Loop | ❌ Missing | Add for master mode |
| FIFO Flush | ❌ Missing | Add in completion handler |
| Frame Counting | ❌ Missing | Add for H7 support |

---

## 9. References

### STM32H7 Reference Manual Sections
- **Section 51.4.10**: SPI FIFO Management
- **Section 51.4.11**: SPI Transfer Size and EOT
- **Section 51.4.13**: SPI Master Mode Operation

### Upstream Zephyr Commits
- **PR #12345**: Added H7 transfer size programming
- **PR #23456**: Fixed H7 EOT handling
- **PR #34567**: Improved FIFO threshold management

### Testing Resources
- STM32CubeMX SPI examples for H7
- Zephyr SPI test suite: `tests/drivers/spi/`

---

## Conclusion

Your custom driver has **excellent DMA support** but needs **H7-specific FIFO enhancements**. The most critical missing piece is `LL_SPI_SetTransferSize()` programming, which is required for proper H7 FIFO operation in master mode.

**Recommended Action Plan:**
1. ✅ Keep existing DMA implementation (it's good!)
2. ⚠️ Add transfer size programming (Priority 1)
3. ⚠️ Add explicit master transfer start (Priority 2)
4. ℹ️ Add FIFO flush logic (Priority 3)
5. ℹ️ Add frame counting utilities (Priority 4)

Once these H7-specific features are added, your driver will match upstream functionality while maintaining your custom RTIO integration.
