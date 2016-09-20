package main;

import java.lang.reflect.Method;
import java.util.ArrayList;

import org.opencv.objdetect.Objdetect;

import processing.core.PApplet;
import processing.core.PVector;

import com.theeyetribe.client.GazeManager;
import com.theeyetribe.client.ICalibrationProcessHandler;
import com.theeyetribe.client.ICalibrationResultListener;
import com.theeyetribe.client.IGazeListener;
import com.theeyetribe.client.ITrackerStateListener;
import com.theeyetribe.client.data.CalibrationResult;
import com.theeyetribe.client.data.GazeData;

public class TheEyeTribe implements IGazeListener, ICalibrationProcessHandler, ITrackerStateListener, ICalibrationResultListener{
	
	PApplet main;
	
	private Method getGazeMethod;
	private Method getKalibrasiPointMethod;
	private Method getKalibrasiEndedMethod;
	private Method getScreenResolution;
	private Method getCalibrationSwitch;
	
	GazeManager gm;
	
	//Kalibrasi
	private PVector titikTitikKalibrasi[];
	private int calibrationAttempts = 0;
	private int titikKalibrasiSekarang = 0;
	private int intervalTitikKalibrasi;
	private int durasiTitikKalibrasi;
	boolean salinHasilKalibrasi = false;
	
	public TheEyeTribe(PApplet theParent) {
		this(theParent, 300, 1200);
	}	
	
	public TheEyeTribe(PApplet main, int intervalTitikKalibrasi, int durasiTitikKalibrasi) {
		// TODO Auto-generated constructor stub
		this.main = main;
		this.intervalTitikKalibrasi = intervalTitikKalibrasi;
		this.durasiTitikKalibrasi = durasiTitikKalibrasi;
		
		try {
			getGazeMethod = main.getClass().getMethod("onGazeUpdate", new Class[]{GazeData.class, boolean.class});
			
		} catch (Exception e) {
			// TODO: handle exception
			System.err.println("Metod onGazeUpdate tidak didefinisikan di Class MataProduction");
		}
		
		try {
			getKalibrasiPointMethod = main.getClass().getMethod("titikKalibrasi", new Class[]{PVector.class, boolean.class});
		} catch (Exception e) {
			System.err.println("Metod titikKalibrasi tidak didefinisikan di Class MataProduction");
		}
		
		try {
			getKalibrasiEndedMethod = main.getClass().getMethod("akhirDariKalibrasi", new Class[]{CalibrationResult.class});
		} catch (Exception e) {
			System.err.println("Metod akhirDariKalibrasi tidak didefinisikan di Class MataProduction");
		}
		
		try {
			getScreenResolution = main.getClass().getMethod("screenResolution", new Class[]{float.class, float.class, float.class, float.class});
			
		} catch (Exception e) {
			// TODO: handle exception
			System.err.println("Metod screenResolution tidak didefinisikan di Class MataProduction");
		}
		
		try {
			getCalibrationSwitch = main.getClass().getMethod("calibrationSwitch", new Class[]{int.class});
		} catch (Exception e) {
			// TODO: handle exception
			System.err.println("Metod calibrationSwitch tidak didefinisikan di Class MataProduction");
		}
		
		
		
		gm = GazeManager.getInstance();
		boolean sukses = gm.activate(GazeManager.ApiVersion.VERSION_1_0, GazeManager.ClientMode.PUSH);
		
		//if(!sukses){
		//	System.err.println("Eye tribe tidak aktif");
		//	return;
		//}
		
		gm.addGazeListener(this);
		gm.addTrackerStateListener(this);
		gm.addCalibrationResultListener(this);
	}
	
	
	public void onGazeUpdate(GazeData gazeData) {
		// TODO Auto-generated method stub
		float screenPhysicalHeight = gm.getScreenPhysicalHeight() * 100;
		float screenPhysicalWidth = gm.getScreenPhysicalWidth() * 100;
		float screenResolutionHeight = gm.getScreenResolutionHeight();
		float screenResolutionWidth = gm.getScreenResolutionWidth();
		
		boolean isCalibrated = gm.isCalibrated();		
		
		try {
			getGazeMethod.invoke(main, new Object[]{gazeData, isCalibrated});
		} catch (Exception e) {
			// TODO: handle exception
			System.err.println("Gaze update tidak bisa jalan karena error");
			System.err.println(e.getMessage());
			e.printStackTrace();
			getGazeMethod = null;
		}
		
		try {
			getScreenResolution.invoke(main, new Object[]{screenPhysicalHeight, screenPhysicalWidth, screenResolutionHeight, screenResolutionWidth});
		} catch (Exception e) {
			// TODO: handle exception
			System.err.println("screen Resolution tidak bisa jalan karena error");
			System.err.println(e.getMessage());
			e.printStackTrace();
			getScreenResolution = null;
		}	
	}
	
	public void kalibrasi(PVector titikTitikKalibrasi[]){
		calibrationAttempts = 0;
		kalibrasi(titikTitikKalibrasi, true);
	}
	
	public void kalibrasi(PVector titikTitikKalibrasi[], boolean mulai){
		this.titikTitikKalibrasi = titikTitikKalibrasi;
		calibrationAttempts++;
		
		if(mulai){
			gm.calibrationStart(titikTitikKalibrasi.length, this);
			//System.out.println("Jumlah titik kalibrasi: "+titikTitikKalibrasi.length);
			//System.out.println("tes :"+titikTitikKalibrasi);
		}else{
			onCalibrationStarted();
		}
	}
	
	private void panggilTitik(int index){
		try{
			getKalibrasiPointMethod.invoke(main, new Object[]{titikTitikKalibrasi[index], true});
		}catch(Exception e){
			System.err.println("Metod titikKalibrasi Error");
			getKalibrasiPointMethod = null;
		}
		
		try {
			Thread.sleep(this.intervalTitikKalibrasi);
		} catch (Exception e) {
			// TODO: handle exception
			System.out.println("Titik Kalibrasi Error");
		}
		
		//Memulai Kalibrasi
		gm.calibrationPointStart((int)titikTitikKalibrasi[index].x, (int)titikTitikKalibrasi[index].y);
		
		try {
			Thread.sleep(this.durasiTitikKalibrasi);
		} catch (Exception e) {
			// TODO: handle exception
			System.err.println("Durasi titik kalibrasi error");
		}
		
		
		gm.calibrationPointEnd();
		try {
			getKalibrasiPointMethod.invoke(main, new Object[]{titikTitikKalibrasi[index], false});
		} catch (Exception e) {
			// TODO: handle exception
			System.err.println("Metod titikKalibrasi error");
			getKalibrasiPointMethod = null;
		}
		
		//gm.onGazeApiResponse(response);
		
	}
	
	public void clearKalibrasi(){
		try {
			gm.calibrationClear();
		} catch (Exception e) {
			// TODO: handle exception
			System.err.println(e.getMessage());
			e.printStackTrace();
		}
	}
	
	
	public void onCalibrationStarted() {
		// TODO Auto-generated method stub
		titikKalibrasiSekarang = 0;
		panggilTitik(titikKalibrasiSekarang);
	}

	
	public void onCalibrationProgress(double progress) {
		// TODO Auto-generated method stub
		titikKalibrasiSekarang++;
		if(titikKalibrasiSekarang >= titikTitikKalibrasi.length)
			return;
		panggilTitik(titikKalibrasiSekarang);
	}

	
	public void onCalibrationProcessing() {
		// TODO Auto-generated method stub
		
	}

	
	public void onCalibrationResult(CalibrationResult calibResult) {
		// TODO Auto-generated method stub
		ArrayList<PVector> kalibrasiLagi = new ArrayList<PVector>();
		for(CalibrationResult.CalibrationPoint cp : calibResult.calibpoints){
			if(cp.state == 1){
				kalibrasiLagi.add(new PVector((float)cp.coordinates.x, (float)cp.coordinates.y));
			}
		}

		if(kalibrasiLagi.size() > 0 && calibrationAttempts < 5){
			PVector[] titikTitik = kalibrasiLagi.toArray(new PVector[kalibrasiLagi.size()]);
			kalibrasi(titikTitik, false);
		}else{
			if(kalibrasiLagi.size() > 0){
				gm.calibrationAbort();
			}
			
			try {
				getKalibrasiEndedMethod.invoke(main, new Object[]{calibResult});
			} catch (Exception e) {
				// TODO: handle exception
				System.err.println("Metod akhirDariKalibrasi error");
			}
		}
	}

	public void onTrackerStateChanged(int trackerState) {
		// TODO Auto-generated method stub
		
	}

	public void onScreenStatesChanged(int screenIndex,
			int screenResolutionWidth, int screenResolutionHeight,
			float screenPhysicalWidth, float screenPhysicalHeight) {
		// TODO Auto-generated method stub
		
	}
	//Ganti Hasil Kalibrasi berdasarkan perubahan depth
	public void gantiHasilKalibrasi(CalibrationResult hasil){
		//onCalibrationChanged(true, calibResult);
		//gm.addCalibrationResultListener(listener);
		//gm.addCalibrationResultListener(this);
		//CalibrationResultListener.this.onCalibrationChanged(isCalibrated, calibResult);
		//gm.addCalibrationResultListener();
		
		gm.showGantiHasilKalibrasi(hasil);	
		salinHasilKalibrasi = true;
	}
	
	public void onCalibrationChanged(boolean isCalibrated,
			CalibrationResult calibResult) {
		// TODO Auto-generated method stub
		System.out.println("Tamplikan hasil perubahan kalibrasi: "+calibResult.averageErrorDegree);
	}
	
	public static class CalibrationResultListener implements ICalibrationResultListener{

		public void onCalibrationChanged(boolean isCalibrated,
				CalibrationResult calibResult) {
			// TODO Auto-generated method stub
			
		}
		
	}
}


