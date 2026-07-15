import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.locks.Lock;

import com.virtenio.driver.device.ADXL345;
import com.virtenio.driver.gpio.GPIO;
import com.virtenio.driver.gpio.GPIOException;
import com.virtenio.driver.gpio.NativeGPIO;
import com.virtenio.driver.spi.NativeSPI;
import com.virtenio.driver.spi.SPIException;
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
import com.virtenio.driver.device.at86rf231.AT86RF231RadioDriver;
import com.virtenio.driver.led.LED;

import com.virtenio.vm.Time;

public class nSensorSingle {
	private int channel = 24;
	private int panID = 0xCAFE;
	
	int[] nodeAddress = {0xBABE, 0xAFFE, 0xBAFE, 0xBEBA};
	
	//Untuk AFFE menggunakan indexNode 1 
	//Untuk BAFE menggunakan indexNode 2
	//Untuk BEBA menggunakan indexNode 3
	int indexNode = 3;
	int alamatNode = nodeAddress[indexNode];
	
	private LED red;
	
	AT86RF231 radio;
	FrameIO fio;
	Shuttle shuttle;
	
	private ADXL345 acclSensor;
	private GPIO accelCs;
	
	Map<Integer, String> buffer = new HashMap<Integer, String>();

	//SN paling kecil yang belum di ACK
	int base = 0; 
	int nextSeqNum;
	int windowSize = 5;

	long startTimer = 0;
	long timeout = 2000; // 2 detik
	int timeoutCount = 0;
	
	private volatile boolean exit = false;
	
	public void resetSequenceNumber() throws Exception {
		nextSeqNum = 0;
	}
	
	public void initializeRadio() throws Exception {
		radio = RadioInit.initRadio();
		radio.setChannel(channel);
		radio.setPANId(panID);
		radio.setShortAddress(alamatNode);
	}
	
	public void initializeFrameIO() throws Exception {
		final RadioDriver radioDriver = new AT86RF231RadioDriver(radio);
		fio = new RadioDriverFrameIO(radioDriver);	
	}
	
	private void initializeACCL() throws Exception {
		accelCs = NativeGPIO.getInstance(20); // init GPIO
		NativeSPI spi = NativeSPI.getInstance(0); // init SPI
		spi.open(ADXL345.SPI_MODE, ADXL345.SPI_BIT_ORDER, ADXL345.SPI_MAX_SPEED); // open SPI
		
		// Inisiasi ADXL345
		acclSensor = new ADXL345(spi,accelCs);
		acclSensor.open();
		acclSensor.setPowerControl(ADXL345.POWER_MODE_NORMAL);
		acclSensor.setDataFormat(ADXL345.DATA_FORMAT_RANGE_16G); 
		acclSensor.setDataRate(ADXL345.DATA_RATE_100HZ);
		acclSensor.setPowerControl(ADXL345.POWER_CONTROL_MEASURE);
	}
	
	public void initializeLED() throws Exception {
		shuttle = Shuttle.getInstance();
		
		red = shuttle.getLED(Shuttle.LED_RED);
		red.open();
	}
	
	public void sendFrameToBS(final FrameIO fio, String mesg, int sn) throws InterruptedException {
		try {
			String message = "SENSE " + mesg;
				
			Frame frame = new Frame(Frame.TYPE_DATA | Frame.ACK_REQUEST | Frame.DST_ADDR_16 | 
									Frame.INTRA_PAN | Frame.SRC_ADDR_16);
							
			frame.setSrcAddr(alamatNode);
			frame.setSrcPanId(panID);
			frame.setDestAddr(nodeAddress[0]);
			frame.setDestPanId(panID);
			frame.setSequenceNumber(sn);
			frame.setPayload(message.getBytes());
			radio.setState(AT86RF231.STATE_TX_ARET_ON);
			fio.transmit(frame);
		} catch (RadioDriverException e) { 
			e.printStackTrace();
		} catch (NoAckException e) { 
		} catch (ChannelBusyException e) { 
		} catch (IOException e) {
		}
	}
		
	// Sensing dan kirim data ke BS
	public void goSensing() throws Exception {
        new Thread() {
            public void run() {
                try {
                    initializeACCL();
                } catch (Exception e) {
                    e.printStackTrace();
                }
 
                short[] valAccl = new short[3];
 
                while (!exit) {
                    try {
                        long getT = Time.currentTimeMillis();
                        acclSensor.getValuesRaw(valAccl, 0);
 
                        float XG = valAccl[0] / 2048.0f;
                        float YG = valAccl[1] / 2048.0f;
                        float ZG = valAccl[2] / 2048.0f;
 
                        float magnitudeG   = (float) Math.sqrt(XG * XG + YG * YG + ZG * ZG);
                        float magnitudeMS2 = magnitudeG * 9.80665f;
 
                        String valStr = nextSeqNum + " " + Integer.toHexString(alamatNode) + " " +
                                stringFormatTime.SFFull(getT) +
                                " X:" + valAccl[0] + " Y:" + valAccl[1] + " Z:" + valAccl[2] +
                                " " + magnitudeG + "g " + magnitudeMS2 + "m/s^2";
 
                        System.out.println(valStr);
                        
                        if (nextSeqNum < base + windowSize) {
                            buffer.put(nextSeqNum, valStr);
                            nextSeqNum++;
                        }
                        Thread.sleep(100);
 
                    } catch (SPIException e) {
                    } catch (GPIOException e) {
                    } catch (InterruptedException e) {
                    }
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
                                sendFrameToBS(fio, data, lastSent);
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
 
	private void startTransmitter (final FrameIO fio, final String mesg, final int sn, final long destADDR) {
		new Thread() {
			@Override
			public void run() {
				boolean isOK = false;
				while (!isOK) {
					try {
						int frameControl = 	Frame.TYPE_DATA | Frame.ACK_REQUEST | Frame.DST_ADDR_16 | 
											Frame.INTRA_PAN | Frame.SRC_ADDR_16;
						
						final Frame frame = new Frame(frameControl);
						
						frame.setSrcAddr(alamatNode);
						frame.setSrcPanId(panID);
						frame.setDestAddr(destADDR);
						frame.setDestPanId(panID);
						frame.setSequenceNumber(sn);
						String message = mesg + " " + Time.currentTimeMillis();
						frame.setPayload(message.getBytes());
						radio.setState(AT86RF231.STATE_TX_ARET_ON);
						fio.transmit(frame); 
						System.out.println("transmitted");
						isOK = true;
					} catch (Exception e) { 
						e.printStackTrace();
					}
				}
			}
		}.start();
	}

	public void cekNodeAktif(final String mesgSplit[], final long destADDR, final long t2) throws Exception {	
		long t1 = Long.parseLong(mesgSplit[2]);
		String message = "001" + " " + t1 + " " + t2; 
		startTransmitter(fio, message, nextSeqNum++, destADDR);
	}
	
	public void sinkronisasiWaktu(String mesgSplit[], long destADDR, long t2) throws Exception { 
		// format pesan: 010 deltaDelay t1
		long deltat3t2;
		Time.setCurrentTimeMillis(
				Long.parseLong(mesgSplit[2]) + 
				Long.parseLong(mesgSplit[1]) + ( deltat3t2 = Time.currentTimeMillis() - t2) );

		// format pesan reply : 010 t2 t3
		String message = "010" + " " + deltat3t2 + " " + (Time.currentTimeMillis());
		System.out.println(stringFormatTime.SFFull(Time.currentTimeMillis()));
		startTransmitter(fio, message, nextSeqNum++, destADDR); 
	}
	
	public void kirimkanWaktu(String mesgSplit[], long destADDR, long t2) throws Exception { 
		
		long t1 = Long.parseLong(mesgSplit[2]);
		String message = "011" + " " + t1 + " " + t2;
		startTransmitter(fio, message, nextSeqNum++, destADDR);
	}
	
	public void dispatch (final Frame f, final long t2) throws Exception {
		new Thread () {
			final Lock lock = new ReentrantLock();
			
			public void run() {
				int code=-1;
				if (f!=null) {
					try {
						byte[] payloadBytes = f.getPayload(); 
						String pesanDiterima = new String(payloadBytes, 0, payloadBytes.length);
						String splitPesan[] = StringUtils.split(pesanDiterima, " ");
				
						if (splitPesan[0].equalsIgnoreCase("001")) {
							code = 1;
						} else if (splitPesan[0].equalsIgnoreCase("010")) {
							code = 2;
						} else if (splitPesan[0].equalsIgnoreCase("011")) {
							code = 3;
						} else if (splitPesan[0].equalsIgnoreCase("100")) {
							code = 4;
						} else if (splitPesan[0].equalsIgnoreCase("110")) {
							code = 5;
						} else if (splitPesan[0].equalsIgnoreCase("111")) {
							code = 6;
						}
						
						switch (code) {
							case 1 : { 
								cekNodeAktif(splitPesan, f.getSrcAddr(), t2); 
								break; 
							}
							case 2 : { 
								sinkronisasiWaktu(splitPesan, f.getSrcAddr(),t2); 
								break; 
							}
							case 3 : { 
								kirimkanWaktu(splitPesan, f.getSrcAddr(), t2); 
								break; 
							}
							case 4 : { 
								nextSeqNum = 0;
							    base = 0;
							    buffer.clear();
							    				
								goSensing(); 
								break; 
							}
							case 5 : { 
								handleACK(splitPesan);
								break; 
							}
							case 6 : { 
								System.out.println("stop"); 
								lock.lock(); 
								try { 
									buffer.clear();
									exit = true; 
								} finally { 
									lock.unlock();
								}
								break; 
							}
						}
					} catch (Exception e) { 
						e.printStackTrace();
					}
				}
			}
		}.start();
	}
	
	private void handleACK(String[] split) {
	    try {
	        int ackNum = Integer.parseInt(split[2]);

	        System.out.println("ACK received for SN=" + ackNum);

	        if (ackNum < base) {
	            return; // abaikan ACK lama
	        }

	        for (int seq = base; seq <= ackNum; seq++) {
	            buffer.remove(seq);
	        }

	        System.out.println("Buffer remove until SN = " + ackNum);

	        base = ackNum + 1;
	        startTimer = Time.currentTimeMillis();

	    } catch (NumberFormatException e) {
	        e.printStackTrace();
	    }
	}
	
	public void run() {
		try {
			initializeLED();
			initializeRadio(); 
			initializeFrameIO();
			resetSequenceNumber();
			red.on();
			startReceiver(fio);
			startTimer();
		} catch (Exception e) { 
			e.printStackTrace(); 
		}
	}
	
	public void startReceiver(final FrameIO fio) {
	    new Thread() {
	        public void run() {
	            Frame frame = new Frame();
	            boolean stop = false;
	            Integer prevSN = -1;

	            while (!stop) {
	                System.out.println("Ready");
	                try {
	                    radio.setState(AT86RF231.STATE_RX_AACK_ON);
	                    fio.receive(frame);
	                    long t2 = Time.currentTimeMillis();
	                    if (frame.getSequenceNumber() != prevSN) {
	                        dispatch(frame, t2);
	                    }
	                } catch (Exception e) {
	                    e.printStackTrace();
	                }
	                prevSN = frame.getSequenceNumber();
	            }
	        }
	    }.start();
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
	                            if (data != null) {
	                                sendFrameToBS(fio, data, seq);
	                            }
	                            seq++;
	                        }
	                        startTimer = now;
	                    }
	                    Thread.sleep(100);
	                } catch (Exception e) {
	                    e.printStackTrace();
	                }
	            }
	        }
	    }.start();
	}
		
	public static void main(String [] args ) throws Exception {
		new nSensorSingle().run();
	}
}