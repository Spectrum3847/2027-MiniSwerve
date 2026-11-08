package frc.spectrumLib.battery;

import java.io.IOException;

/**
 * Byte pipe to a PN532 in HSU (UART) mode. Kept separate from {@link Pn532} so the driver can run
 * over jSerialComm on a laptop or the SystemCore, over WPILib's SerialPort, or over a fake in
 * tests.
 */
public interface Pn532Transport extends AutoCloseable {

    void write(byte[] data) throws IOException;

    /**
     * Reads up to {@code buf.length} bytes, waiting at most {@code timeoutMs} for the first one.
     *
     * @return bytes read, 0 on timeout
     */
    int read(byte[] buf, int timeoutMs) throws IOException;

    /** Drops anything already received. */
    void discardInput() throws IOException;

    @Override
    void close();
}
