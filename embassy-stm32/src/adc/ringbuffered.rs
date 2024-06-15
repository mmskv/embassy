use embassy_futures::select::Either;
use embassy_hal_internal::PeripheralRef;
use stm32_metapac::adc::vals::{Dmngt, Exten, Pcsel};

use super::{Adc, AdcPin, Error, Instance};
use crate::dma::ReadableRingBuffer;
use core::future::poll_fn;
use core::mem;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

pub struct RingBufferedAdc<'d, T: Instance> {
    _peri: PeripheralRef<'d, T>,
    ring_buf: ReadableRingBuffer<'d, u16>,
}

impl<'d, T: Instance, RxDma: super::RxDma<T>> Adc<'d, T, RxDma> {
    /// Turn the `RxDma` into a buffered uart which can continously receive in the background
    /// without the possibility of losing bytes. The `dma_buf` is a buffer registered to the
    /// DMA controller, and must be large enough to prevent overflows.
    pub fn into_ring_buffered(self, dma_buf: &'d mut [u16]) -> RingBufferedAdc<'d, T> {
        assert!(!dma_buf.is_empty() && dma_buf.len() <= 0xFFFF); // TODO

        let request = self.dma.request();
        let opts = Default::default();

        // Safety: we forget the struct before this function returns.
        let dma = unsafe { self.dma.clone_unchecked() };
        let _peri = unsafe { self.adc.clone_unchecked() };

        let ring_buf =
            unsafe { ReadableRingBuffer::new(dma, request, T::regs().dr().as_ptr() as *mut u16, dma_buf, opts) };

        // Don't disable the clock
        mem::forget(self);

        RingBufferedAdc { _peri, ring_buf }
    }
}

impl<'d, T: Instance> RingBufferedAdc<'d, T> {
    /// Clear the ring buffer and start receiving in the background
    pub fn start(&mut self) {
        // Clear the ring buffer so that it is ready to receive data
        self.ring_buf.clear();

        self.start_adc();
    }

    pub fn configure_pin<P>(&mut self, pin: &mut P)
    where
        P: AdcPin<T>,
        P: crate::gpio::sealed::Pin,
    {
        pin.set_as_analog();
        let channel = pin.channel();
        {
            let sample_time = stm32_metapac::adc::vals::SampleTime::CYCLES1_5; // Can't set to 1.5
            let sample_time = sample_time.into();
            if channel <= 9 {
                T::regs().smpr(0).modify(|reg| reg.set_smp(channel as _, sample_time));
            } else {
                T::regs()
                    .smpr(1)
                    .modify(|reg| reg.set_smp((channel - 10) as _, sample_time));
            }
        };

        {
            T::regs().cfgr2().modify(|w| w.set_lshift(0));
            T::regs()
                .pcsel()
                .write(|w| w.set_pcsel(channel as _, Pcsel::PRESELECTED));
        }

        T::regs().sqr1().write(|reg| {
            reg.set_sq(0, channel);
            reg.set_l(0);
        });
    }

    /// Start adc background receive
    fn start_adc(&mut self) {
        // fence before starting DMA.
        compiler_fence(Ordering::SeqCst);

        // start the dma controller
        self.ring_buf.start();

        let r = T::regs();
        // Reset EOS and EOC
        // TODO not sure if needed
        r.isr().modify(|reg| {
            reg.set_eos(true);
            reg.set_eoc(true);
        });
        // DMA configuration
        r.cfgr().modify(|reg| {
            reg.set_cont(true);
            reg.set_dmngt(Dmngt::DMA_CIRCULAR);
            reg.set_res(stm32_metapac::adc::vals::Res::BITS16);
        });
        r.cr().modify(|reg| {
            reg.set_adstart(true);
        });
    }

    /// Read bytes that are readily available in the ring buffer.
    /// If no bytes are currently available in the buffer the call waits until the some
    /// bytes are available (at least one byte and at most half the buffer size)
    ///
    /// Background receive is started if `start()` has not been previously called.
    ///
    /// Receive in the background is terminated if an error is returned.
    /// It must then manually be started again by calling `start()` or by re-calling `read()`.
    pub async fn read(&mut self, buf: &mut [u16]) -> Result<usize, Error> {
        let r = T::regs();

        let overrun = r.isr().read().ovr();
        if overrun {
            panic!("Got overrun: {}", overrun);
        }

        loop {
            match self.ring_buf.read(buf) {
                Ok((0, _)) => {}
                Ok((len, _)) => {
                    return Ok(len);
                }
                Err(e) => {
                    panic!("PANIC OVERRUN");
                }
            }

            self.wait_for_data().await
        }
    }

    /// Wait for uart idle or dma half-full or full
    async fn wait_for_data(&mut self) {
        compiler_fence(Ordering::SeqCst);

        let mut dma_init = false;
        // Future which completes when there is dma is half full or full
        let dma = poll_fn(|cx| {
            self.ring_buf.set_waker(cx.waker());

            let status = match dma_init {
                false => Poll::Pending,
                true => Poll::Ready(()),
            };

            dma_init = true;
            status
        });

        dma.await
    }
}

// impl<T: BasicInstance> Drop for RingBufferedAdc<'_, T> {
//     fn drop(&mut self) {
//         self.teardown_uart();
//
//         T::disable();
//     }
// }
// /// Return an error result if the Sr register has errors
// fn check_for_errors(s: Sr) -> Result<(), Error> {
//     if s.pe() {
//         Err(Error::Parity)
//     } else if s.fe() {
//         Err(Error::Framing)
//     } else if s.ne() {
//         Err(Error::Noise)
//     } else if s.ore() {
//         Err(Error::Overrun)
//     } else {
//         Ok(())
//     }
// }
//
// /// Clear IDLE and return the Sr register
// fn clear_idle_flag(r: Regs) -> Sr {
//     // SAFETY: read only and we only use Rx related flags
//
//     let sr = sr(r).read();
//
//     // This read also clears the error and idle interrupt flags on v1.
//     unsafe { rdr(r).read_volatile() };
//     clear_interrupt_flags(r, sr);
//
//     r.cr1().modify(|w| w.set_idleie(true));
//
//     sr
// }

// impl<T> embedded_io_async::ErrorType for RingBufferedAdc<'_, T>
// where
//     T: Instance,
// {
//     type Error = Error;
// }
//
// impl<T> embedded_io_async::Read for RingBufferedAdc<'_, T>
// where
//     T: Instance,
// {
//     async fn read(&mut self, buf: &mut [u16]) -> Result<usize, Error> {
//         self.read(buf).await
//     }
// }
