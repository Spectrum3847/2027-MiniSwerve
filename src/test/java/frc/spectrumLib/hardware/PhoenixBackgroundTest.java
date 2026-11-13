package frc.spectrumLib.hardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusCode;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class PhoenixBackgroundTest {
    @Test
    void retriesAndStaysPendingUntilTheLastCallOfANameFinishes() throws Exception {
        CountDownLatch gate = new CountDownLatch(1);
        CountDownLatch done = new CountDownLatch(2);
        AtomicInteger tries = new AtomicInteger();
        PhoenixBackground.run(
                "test",
                () -> {
                    try {
                        gate.await(1, TimeUnit.SECONDS);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                    return StatusCode.OK;
                },
                ok -> done.countDown());
        PhoenixBackground.run(
                "test",
                () -> tries.incrementAndGet() < 3 ? StatusCode.TxTimeout : StatusCode.OK,
                ok -> done.countDown());
        assertTrue(PhoenixBackground.pending("test"));
        gate.countDown();
        assertTrue(done.await(2, TimeUnit.SECONDS));
        Thread.sleep(20);
        assertFalse(PhoenixBackground.pending("test"));
        assertEquals(3, tries.get());
        assertTrue(PhoenixBackground.result("test").isOK());
    }

    @Test
    void aThrowingCallStillFinishesAndCanBeQueuedAgain() throws Exception {
        CountDownLatch done = new CountDownLatch(1);
        boolean[] result = {true};
        PhoenixBackground.run(
                "throws",
                () -> {
                    throw new IllegalStateException("bad config");
                },
                ok -> {
                    result[0] = ok;
                    done.countDown();
                });
        assertTrue(done.await(2, TimeUnit.SECONDS));
        Thread.sleep(20);
        assertFalse(result[0]);
        assertFalse(PhoenixBackground.pending("throws"), "not stuck pending");
        assertFalse(PhoenixBackground.result("throws").isOK());
        assertTrue(PhoenixBackground.thrown("throws").contains("bad config"));
    }

    @Test
    void aCallGivenOneTryIsNotRetried() throws Exception {
        CountDownLatch done = new CountDownLatch(1);
        AtomicInteger tries = new AtomicInteger();
        boolean[] result = {true};
        PhoenixBackground.run(
                "oneTry",
                () -> {
                    tries.incrementAndGet();
                    return StatusCode.TxTimeout;
                },
                ok -> {
                    result[0] = ok;
                    done.countDown();
                },
                1);
        assertTrue(done.await(2, TimeUnit.SECONDS));
        assertEquals(1, tries.get());
        assertFalse(result[0]);
    }
}
