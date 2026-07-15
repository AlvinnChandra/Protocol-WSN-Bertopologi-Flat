import java.io.IOException;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.locks.Lock;
import java.util.Map;
import java.util.HashMap;

import com.virtenio.driver.i2c.NativeI2C;
import com.virtenio.driver.i2c.I2C;
import com.virtenio.driver.i2c.I2CException;
import com.virtenio.driver.device.ADT7410;
import com.virtenio.io.ChannelBusyException;
import com.virtenio.io.NoAckException;
import com.virtenio.misc.StringUtils;
import com.virtenio.preon32.examples.common.RadioInit;
import com.virtenio.preon32.shuttle.Shuttle;
import com.virtenio.radio.ieee_802_15_4.RadioDriver;
import com.virtenio.radio.ieee_802_15_4.Frame;
import com.virtenio.radio.ieee_802_15_4.FrameIO;
import com.virtenio.radio.ieee_802_15_4.RadioDriverFrameIO;
import com.virtenio.radio.RadioDriverException;

import com.virtenio.driver.device.at86rf231.*;
import com.virtenio.driver.led.LED;

import com.virtenio.vm.Time;

public class nSensorRelay {
    int channel = 24;
    int panID   = 0xCAFE;

    int[] nodeAddress = {0xBABE, 0xAFFE, 0xBAFE, 0xBEBA};

    int indexNode = 2;
    int alamatNode = nodeAddress[indexNode];

    int nextHop = nodeAddress[indexNode + 1];
    int prevHop = nodeAddress[indexNode - 1];

    private LED red;

    AT86RF231 radio;
    FrameIO fio;
    Shuttle shuttle;

    private ADT7410 sensorSuhu;
    private NativeI2C i2c;

    int sn;
    int nextSeqNum;

    Map<Integer, String> buffer = new HashMap<Integer, String>();

    int base = 0;

    long startTimer = 0;
    long timeout = 2000;
    int timeoutCount = 0;

    int windowSize = 5;

    private volatile boolean exit = false;

    // mutex
    private final ReentrantLock lock = new ReentrantLock();

    public void resetSequenceNumber() throws Exception {
        sn = 0;
        nextSeqNum = 0;
    }

    public void initRadio() throws Exception {
        radio = RadioInit.initRadio();
        radio.setChannel(channel);
        radio.setPANId(panID);
        radio.setShortAddress(alamatNode);
    }

    public void initFrameIO() throws Exception {
        final RadioDriver radioDriver = new AT86RF231RadioDriver(radio);
        fio = new RadioDriverFrameIO(radioDriver);
    }

    public void initTEMPERATURE() throws Exception {
        i2c = NativeI2C.getInstance(1);
        i2c.open(I2C.DATA_RATE_400);
        sensorSuhu = new ADT7410(i2c, ADT7410.ADDR_0, null, null);
        sensorSuhu.open();
        sensorSuhu.setMode(ADT7410.CONFIG_MODE_CONTINUOUS);
    }

    public void initLED() throws Exception {
        shuttle = Shuttle.getInstance();
        red = shuttle.getLED(Shuttle.LED_RED);
        red.open();
    }

    public void cekNodeAktif(String[] split, long t2) throws Exception {
        long t1 = Long.parseLong(split[2]);
        long t3 = Time.currentTimeMillis();
        String msg = "001 " + t1 + " " + t2 + " " + t3;
        sendToBS(msg, sn++, alamatNode);
    }

    public void sinkronisasiWaktu(String[] split, long t2) throws Exception {
        long delta     = Long.parseLong(split[2]);
        long t1        = Long.parseLong(split[4]);
        long deltat3t2 = Time.currentTimeMillis() - t2;
        Time.setCurrentTimeMillis(t1 + delta + deltat3t2);
        long t3 = Time.currentTimeMillis();
        System.out.println(stringFormatTime.SFFull(t3));
        String msg = "010 " + deltat3t2 + " 0 " + t3;
        sendToBS(msg, sn++, alamatNode);
    }

    public void kirimkanWaktu(String[] split, long t2) throws Exception {
        long t1 = Long.parseLong(split[2]);
        long t3 = Time.currentTimeMillis();
        String msg = "011 " + t1 + " " + t2 + " " + t3;
        sendToBS(msg, sn++, alamatNode);
    }

    public void dispatch(final Frame f, final long t2) throws Exception {
        new Thread() {
            final Lock lock = new ReentrantLock();
            public void run() {
                int code = -1;
                try {
                    if (f.getSrcAddr() == alamatNode) {
                    	return;
                    }

                    byte[] payloadBytes = f.getPayload();
                    String mesgRecv = new String(payloadBytes, 0, payloadBytes.length);
                    String[] mesgSplit = StringUtils.split(mesgRecv, " ");

                    if (mesgSplit == null || mesgSplit.length == 0) {
                    	return;
                    }

                    boolean dariArahBS = (f.getSrcAddr() == nextHop);
                    if (!dariArahBS) {
                        relayToNextHop(mesgRecv, sn++, f.getSrcAddr());
                        return;
                    }

                    if (mesgSplit[0].equalsIgnoreCase("SENSE")) {
                    	return;
                    }

                    if (mesgSplit[0].equalsIgnoreCase("001")) {
                    	code = 1;
                    } else if (mesgSplit[0].equalsIgnoreCase("010")) {
                    	code = 2;
                    } else if (mesgSplit[0].equalsIgnoreCase("011")) {
                    	code = 3;
                    } else if (mesgSplit[0].equalsIgnoreCase("100")) {
                    	code = 4;
                    } else if (mesgSplit[0].equalsIgnoreCase("110")) {
                    	code = 5;
                    } else if (mesgSplit[0].equalsIgnoreCase("111")) {
                    	code = 6;
                    }

                    if (code >= 1 && code <= 4 || code == 6) {
                        relayToPrevHop(mesgRecv, sn++);
                    }

                    switch (code) {
                        case 1: {
                        	cekNodeAktif(mesgSplit, t2);
                        	break;
                        }
                        case 2: {
                        	sinkronisasiWaktu(mesgSplit, t2);
                        	break;
                        }
                        case 3: {
                        	kirimkanWaktu(mesgSplit, t2);
                        	break;
                        }
                        case 4: {
                            nextSeqNum = 0;
                            base = 0;
                            buffer.clear();
                            goSensing();
                            break;
                        }
                        case 5: {
                            long targetAddrLong = Long.parseLong(mesgSplit[1], 16);
                            relayToPrevHop(mesgRecv, sn++);
                            if (targetAddrLong == alamatNode) {
                            	handleACK(mesgSplit);
                            }
                            break;
                        }
                        case 6: {
                            System.out.println("stop");
                            lock.lock();
                            try {
                            	buffer.clear();
                                exit = true;
                                if (sensorSuhu != null) {
                                	sensorSuhu.close();
                                }
                                if (i2c != null) {
                                	i2c.close();
                                }
                            } catch (Exception e) {
                            } finally { lock.unlock(); }
                            break;
                        }
                    }
                } catch (Exception e) {}
            }
        }.start();
    }

    private void handleACK(String[] split) {
        try {
            int ackNum = Integer.parseInt(split[2]);
            System.out.println("ACK received for SN=" + ackNum);
            
            if (ackNum < base) {
            	return;
            }
            
            for (int seq = base; seq <= ackNum; seq++) {
            	buffer.remove(seq);
            }
            
            System.out.println("Buffer remove until SN = " + ackNum);
            base = ackNum + 1;
            startTimer = Time.currentTimeMillis();
        } catch (NumberFormatException e) {}
    }

    public void goSensing() throws Exception {
     	new Thread() {
             public void run() {
                 while (!exit) {
                     try {
                         long waktu = Time.currentTimeMillis();
                         float suhu = sensorSuhu.getTemperatureCelsius();
                         String data = nextSeqNum + " " + Integer.toHexString(alamatNode) + " " +
                                 stringFormatTime.SFFull(waktu) + " TEMP:" + suhu + "°C";
                         System.out.println(data);
  
                         if (nextSeqNum < base + windowSize) {
                             buffer.put(nextSeqNum, data); 
                             nextSeqNum++;                
                         }
                         Thread.sleep(1000);
                     } catch (I2CException e) {
                     } catch (InterruptedException e) {}
                 }
                 System.out.println("Keluar goSense");
             }
         }.start();
         
         new Thread() {
             public void run() {
                 int lastSent = 0;
                 while (!exit) {
                     try {
                         if (lastSent < nextSeqNum && lastSent < base + windowSize) {
                             String data = buffer.get(lastSent);
                             if (data != null) {
                            	 timeoutCount++;
                             	 System.out.println("TIMEOUT ke-" + timeoutCount + ", GBN dari SN=" + base);
                                 sendSenseToNextHop("SENSE " + data, lastSent);
                                 if (base == lastSent) {
                                     startTimer = Time.currentTimeMillis();
                                 }
                                 lastSent++;
                             }
                         }
                         Thread.sleep(10);
                     } catch (InterruptedException e) {}
                 }
             }
         }.start();
     }

    private void sendSenseToNextHop(final String msg, final int snVal) throws InterruptedException {
        boolean ok = false;
        while (!ok) {
            try {
                Frame frame = new Frame(Frame.TYPE_DATA | Frame.ACK_REQUEST
                        | Frame.DST_ADDR_16 | Frame.INTRA_PAN | Frame.SRC_ADDR_16);
                frame.setSrcAddr(alamatNode);
                frame.setSrcPanId(panID);
                frame.setDestAddr(nextHop);
                frame.setDestPanId(panID);
                frame.setSequenceNumber(snVal);
                frame.setPayload(msg.getBytes());
                lock.lock();
                try {
                    radio.setState(AT86RF231.STATE_TX_ARET_ON);
                    fio.transmit(frame);
                } finally {
                    lock.unlock();
                }
                ok = true;
            } catch (RadioDriverException e) {
            } catch (NoAckException e) {
            } catch (ChannelBusyException e) {
            } catch (IOException e) {}
        }
    }

    public void relayToPrevHop(final String msg, final int snVal) {
        new Thread() {
            public void run() {
                boolean ok = false;
                while (!ok) {
                    try {
                        Frame frame = new Frame(Frame.TYPE_DATA | Frame.ACK_REQUEST
                                | Frame.DST_ADDR_16 | Frame.INTRA_PAN | Frame.SRC_ADDR_16);
                        frame.setSrcAddr(alamatNode);
                        frame.setSrcPanId(panID);
                        frame.setDestAddr(prevHop);
                        frame.setDestPanId(panID);
                        frame.setSequenceNumber(snVal);
                        frame.setPayload(msg.getBytes());
                        lock.lock();
                        try {
                            radio.setState(AT86RF231.STATE_TX_ARET_ON);
                            fio.transmit(frame);
                        } finally {
                            lock.unlock();
                        }
                        System.out.println("relay to " + Integer.toHexString(prevHop) + ":" + msg);
                        ok = true;
                    } catch (RadioDriverException e) {
                    } catch (NoAckException e) {
                    } catch (ChannelBusyException e) {
                    } catch (IOException e) {}
                }
            }
        }.start();
    }

    public void relayToNextHop(final String msg, final int snVal, final long originalSrcAddr) {
        new Thread() {
            public void run() {
                boolean ok = false;
                while (!ok) {
                    try {
                        Frame frame = new Frame(Frame.TYPE_DATA | Frame.ACK_REQUEST
                                | Frame.DST_ADDR_16 | Frame.INTRA_PAN | Frame.SRC_ADDR_16);
                        frame.setSrcAddr(originalSrcAddr);
                        frame.setSrcPanId(panID);
                        frame.setDestAddr(nextHop);
                        frame.setDestPanId(panID);
                        frame.setSequenceNumber(snVal);
                        frame.setPayload(msg.getBytes());
                        lock.lock();
                        try {
                            radio.setState(AT86RF231.STATE_TX_ARET_ON);
                            fio.transmit(frame);
                        } finally {
                            lock.unlock();
                        }
                        System.out.println("relay to " + Integer.toHexString(nextHop) +
                                " (srcAddr=" + Long.toHexString(originalSrcAddr) + "): " + msg);
                        ok = true;
                    } catch (RadioDriverException e) {
                    } catch (NoAckException e) {
                    } catch (ChannelBusyException e) {
                    } catch (Exception e) { 
                    	ok = true; 
                    }
                }
            }
        }.start();
    }

    private void sendToBS(final String msg, final int snVal, final long srcAddr) {
        new Thread() {
            public void run() {
                boolean ok = false;
                while (!ok) {
                    try {
                        Frame frame = new Frame(Frame.TYPE_DATA | Frame.ACK_REQUEST
                                | Frame.DST_ADDR_16 | Frame.INTRA_PAN | Frame.SRC_ADDR_16);
                        frame.setSrcAddr(srcAddr);
                        frame.setSrcPanId(panID);
                        frame.setDestAddr(nextHop);
                        frame.setDestPanId(panID);
                        frame.setSequenceNumber(snVal);
                        frame.setPayload(msg.getBytes());
                        lock.lock();
                        try {
                            radio.setState(AT86RF231.STATE_TX_ARET_ON);
                            fio.transmit(frame);
                        } finally {
                            lock.unlock();
                        }
                        System.out.println("sent to " + Integer.toHexString(nextHop) + ": " + msg);
                        ok = true;
                    } catch (NoAckException e) {
                    } catch (ChannelBusyException e) {
                    } catch (Exception e) { ok = true; }
                }
            }
        }.start();
    }

    public void run() {
        try {
            initLED();
            initRadio();
            initFrameIO();
            initTEMPERATURE();
            resetSequenceNumber();
            red.on();
            
            new Thread() {
                public void run() { 
                	startReceiver(fio); 
                }
            }.start();
            
            startTimer();
        } catch (Exception e) {}
    }

    public void startReceiver(final FrameIO fio) {
        Frame frame = new Frame();
        int prevSN = -1;
        long prevSrcAddr = -1;
        while (true) {
            System.out.println("ready " + Integer.toHexString(alamatNode));
            try {
                lock.lock();
                try {
                    radio.setState(AT86RF231.STATE_RX_AACK_ON);
                } finally {
                    lock.unlock();
                }
                fio.receive(frame);
                long t2 = Time.currentTimeMillis();
                System.out.println("received");

                long curSrcAddr = frame.getSrcAddr();
                int curSN = frame.getSequenceNumber();

                if (curSN != prevSN || curSrcAddr != prevSrcAddr) {
                    dispatch(frame, t2);
                }
                prevSN = curSN;
                prevSrcAddr = curSrcAddr;
            } catch (Exception e) {}
        }
    }

    private void startTimer() {
        new Thread() {
            public void run() {
                while (true) {
                    try {
                        long now = Time.currentTimeMillis();
                        if (!buffer.isEmpty() && (now - startTimer > timeout)) {
                        	timeoutCount++;
                        	System.out.println("TIMEOUT ke-" + timeoutCount + ", GBN dari SN=" + base);
                            int seq = base;
                            while (seq < nextSeqNum) {
                                String data = buffer.get(seq);
                                if (data != null) sendSenseToNextHop("SENSE " + data, seq);
                                seq++;
                            }
                            startTimer = now;
                        }
                        Thread.sleep(100);
                    } catch (Exception e) {}
                }
            }
        }.start();
    }

    public static void main(String[] args) throws Exception {
        new nSensorRelay().run();
    }
}