package frc.spectrumLib.battery;

import com.fazecast.jSerialComm.SerialPort;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Arrays;
import java.util.Optional;

/** {@link Pn532Transport} over jSerialComm. Works on Windows (COMn) and Linux (/dev/ttyUSBn). */
public class SerialTransport implements Pn532Transport {

    /** WCH CH340/CH341, the USB-serial chip on the reader we have. */
    public static final int CH340_VID = 0x1A86;

    public static final int BAUD = 115200;

    static {
        preloadWindowsNative();
    }

    private final SerialPort port;

    /**
     * jSerialComm 2.11.0 extracted its ARM64 DLL on our x64 Windows laptop and marked it read-only,
     * so every later run failed with "Can't load ARM 64-bit .dll". Extract the DLL for the running
     * JVM ourselves and point jSerialComm at it before its class loads.
     */
    private static void preloadWindowsNative() {
        if (System.getProperty("jSerialComm.library.path") != null) return;
        if (!System.getProperty("os.name", "").startsWith("Windows")) return;
        String arch =
                switch (System.getProperty("os.arch", "")) {
                    case "amd64", "x86_64" -> "x86_64";
                    case "aarch64" -> "aarch64";
                    case "x86" -> "x86";
                    default -> null;
                };
        if (arch == null) return;
        try (InputStream in =
                SerialPort.class.getResourceAsStream("/Windows/" + arch + "/jSerialComm.dll")) {
            if (in == null) return;
            Path dir =
                    Path.of(
                            System.getProperty("java.io.tmpdir"),
                            "spectrum-jSerialComm-2.11.0",
                            arch);
            Path dll = dir.resolve("jSerialComm.dll");
            if (!Files.exists(dll)) {
                Files.createDirectories(dir);
                Path tmp = Files.createTempFile(dir, "jSerialComm", ".tmp");
                Files.copy(in, tmp, StandardCopyOption.REPLACE_EXISTING);
                Files.move(tmp, dll, StandardCopyOption.ATOMIC_MOVE);
            }
            System.setProperty("jSerialComm.library.path", dir.toString());
        } catch (IOException e) {
            // Fall back to jSerialComm's own extraction.
        }
    }

    public SerialTransport(String portName) throws IOException {
        port = SerialPort.getCommPort(portName);
        port.setComPortParameters(BAUD, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
        port.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
        port.setComPortTimeouts(SerialPort.TIMEOUT_NONBLOCKING, 0, 0);
        if (!port.openPort()) {
            throw new IOException("could not open " + portName + " (in use, or no permission?)");
        }
    }

    /** First port whose USB VID is a CH340, else the only ttyUSB/ttyACM/COM port, if any. */
    public static Optional<String> findReaderPort() {
        SerialPort[] ports = SerialPort.getCommPorts();
        for (SerialPort p : ports) {
            if (p.getVendorID() == CH340_VID) return Optional.of(p.getSystemPortPath());
        }
        SerialPort[] usb =
                Arrays.stream(ports)
                        .filter(p -> p.getSystemPortName().matches("(?i)ttyUSB\\d+|ttyACM\\d+"))
                        .toArray(SerialPort[]::new);
        return usb.length == 1 ? Optional.of(usb[0].getSystemPortPath()) : Optional.empty();
    }

    /** One line per serial port, for the {@code ports} command. */
    public static String describePorts() {
        StringBuilder sb = new StringBuilder();
        for (SerialPort p : SerialPort.getCommPorts()) {
            sb.append(
                    String.format(
                            "%-16s VID %04X PID %04X  %s%n",
                            p.getSystemPortPath(),
                            p.getVendorID() & 0xFFFF,
                            p.getProductID() & 0xFFFF,
                            p.getPortDescription()));
        }
        return sb.length() == 0 ? "(no serial ports)\n" : sb.toString();
    }

    @Override
    public void write(byte[] data) throws IOException {
        int n = port.writeBytes(data, data.length);
        if (n != data.length)
            throw new IOException("serial write failed (" + n + "/" + data.length + ")");
    }

    /**
     * Polls instead of using jSerialComm's read timeouts. On Linux those map to termios VTIME,
     * which counts in 100 ms steps, so the sub-100 ms timeouts {@link Pn532} asks for became "don't
     * wait" and the reader's ACK was missed (seen on the Pi: every command after the first failed
     * with "no ACK"). Windows had honored them, which is why the laptop tests passed.
     */
    @Override
    public int read(byte[] buf, int timeoutMs) throws IOException {
        long deadline = System.nanoTime() + timeoutMs * 1_000_000L;
        while (true) {
            int available = port.bytesAvailable();
            if (available < 0) throw new IOException("serial read failed (reader unplugged?)");
            if (available > 0) {
                int n = port.readBytes(buf, Math.min(available, buf.length));
                if (n < 0) throw new IOException("serial read failed (reader unplugged?)");
                return n;
            }
            if (System.nanoTime() >= deadline) return 0;
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new IOException("interrupted waiting for reader", e);
            }
        }
    }

    @Override
    public void discardInput() {
        int n = port.bytesAvailable();
        if (n > 0) port.readBytes(new byte[n], n);
    }

    @Override
    public void close() {
        port.closePort();
    }
}
