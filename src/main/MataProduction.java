package main;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Vector;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadPoolExecutor;

import javax.print.Doc;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.opencv.core.Mat;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import com.theeyetribe.client.GazeManager;
import com.theeyetribe.client.GazeUtils;
import com.theeyetribe.client.ICalibrationResultListener;
import com.theeyetribe.client.data.CalibrationResult;
import com.theeyetribe.client.data.CalibrationResult.CalibrationPoint;
import com.theeyetribe.client.data.GazeData;
import com.theeyetribe.client.data.Point2D;
import com.theeyetribe.client.data.Point3D;

import processing.core.PApplet;
import processing.core.PVector;

public class MataProduction extends PApplet {

	float screenMataKiriX, screenMataKiriY, screenMataKananX, screenMataKananY;
	float screenMataKiriXNdc, screenMataKiriYNdc, screenMataKananXNdc,
			screenMataKananYNdc;
	float screenPisikMataKiriXNdc, screenPisikMataKiriYNdc,
			screenPisikMataKananXNdc, screenPisikMataKananYNdc;
	float pisikMataKiriX, pisikMataKiriY, pisikMataKananX, pisikMataKananY;
	float screenPhysicalHeight, screenPhysicalWidth, screenResolutionHeight,
			screenResolutionWidth;
	float normalizeMataKiriX, normalizeMataKiriY, normalizeMataKananX,
			normalizeMataKananY;
	float ndcMataKiriX, ndcMataKiriY, ndcMataKananX, ndcMataKananY;
	float ndc2DcMataKiriX, ndc2DcMataKiriY, ndc2DcMataKananX, ndc2DcMataKananY;
	float ndc2PhysicalMataKiriX, ndc2PhysicalMataKiriY, ndc2PhysicalMataKananX,
			ndc2PhysicalMataKananY;
	float wMataKiriX, wMataKiriY, wMataKananX, wMataKananY;
	float ukuranPupilKiri, ukuranPupilKanan;
	float sudut;
	float jarakUser, jarakUserTrigonometri, jarakLurus, tinggiMata;
	float dcNdcMataKiriX, dcNdcMataKiriY, dcNdcMataKananX, dcNdcMataKananY;
	float ipdCmKameraO, ipdCmKameraP, ipdPixelLayar, ipdPixelKamera, ipdPixelO,
			ipdPixelP;
	float RadToDeg = (float) (180 / Math.PI);
	float latestNormalizeDistance, latestAngle;
	float tinggiMataKiri2Meja, tinggiMataKanan2Meja;
	float ndc2Dc3DGazeOX, ndc2Dc3DGazeOY, ndc2Dc3DGazePX, ndc2Dc3DGazePY;;
	float ndc2DcP1X, ndc2DcP1Y;
	float ndc2DcP3X, ndc2DcP3Y;
	float ndc2Dc3DGazeOXNew, ndc2Dc3DGazeOYNew;
	float ndcKalibMataKiriX, ndcKalibMataKiriY, ndcKalibMataKananX,
			ndcKalibMataKananY;
	float ndcKalib2PhysicalMataKiriX, ndcKalib2PhysicalMataKiriY,
			ndcKalib2PhysicalMataKananX, ndcKalib2PhysicalMataKananY;
	float ndc2Dc3DGazeX, ndc2Dc3DGazeY;
	
	double zDisparity;
	double hasilKalibrasiAverageErrorDegree;
	
	int plane = 0;
	int point = 0;
	int xPlane = 0;
	int yPlane = 0;
	int zPlane = 0;
	int depthSebelumKalibrasi = 0;
	int jmlHasilKalibrasi;

	int durasiRecord = 0;

	Point2D ndcKameraMataKiri, ndcKameraMataKanan, realWorlKameraKiri,
			realWorlKameraKanan, pixelKameraKiri, pixelKameraKanan;
	Point2D kameraNdcMataKiri, kameraNdcMataKanan, dataAsalMataKiri,
			dataAsalMataKanan;
	Point2D ndcP1, ndcP3;
	Point2D ndcChangeHeadZ;

	Point3D P1O, P1P, P2, P3O, P3P, P4;
	Point3D P13O, P43O, P21O, P13P, P43P, P21P;
	Point3D PaP, PbP, PaO, PbO;
	Point3D PmO, PmP;
	Point3D ndcGazeO, ndcGazeP;
	Point3D changeHeadZ;
	Point3D ndcGaze;

	TheEyeTribe eyeTribe;
	GazeUtils gazeUtils;

	ArrayList<PVector> tracking;

	PVector gazeData;
	PVector rawMataKiri;
	PVector rawMataKanan;

	// Variabel Kalibrasi
	int kalibrasi = 0;
	PVector titikKalibrasi = null;
	int mulaiKalibrasi = 0;
	int durasiKalibrasi = 1200;
	int intervalKalibrasi = 300;
	String hasilKalibrasi;
	boolean isCalibrated = false;
	boolean record = false;
	boolean eksperimenBoolean = false;
	String infoRecord = "Tidak";

	int calibResult = 0;
	CalibrationResult calibrationResult, calibrationResult1,
			calibrationResult2, calibrationResult3;
	int cekPengirimanDataKalibrasi = 1000;

	public int getCekPengirimanDataKalibrasi() {
		return cekPengirimanDataKalibrasi;
	}

	public void setCekPengirimanDataKalibrasi(int cekPengirimanDataKalibrasi) {
		this.cekPengirimanDataKalibrasi = cekPengirimanDataKalibrasi;
	}

	public void settings() {
		fullScreen(P3D);
		tracking = new ArrayList<PVector>();

		// size(300, 250);
		eyeTribe = new TheEyeTribe(this, intervalKalibrasi, durasiKalibrasi);
		gazeUtils = new GazeUtils();
		ndcKameraMataKiri = new Point2D();
		ndcKameraMataKanan = new Point2D();
		realWorlKameraKanan = new Point2D();
		realWorlKameraKiri = new Point2D();
		pixelKameraKiri = new Point2D();
		pixelKameraKanan = new Point2D();
		kameraNdcMataKiri = new Point2D();
		kameraNdcMataKanan = new Point2D();
		dataAsalMataKiri = new Point2D();
		dataAsalMataKanan = new Point2D();
		ndcP1 = new Point2D();
		ndcP3 = new Point2D();
		ndcChangeHeadZ = new Point2D();
		
		
		P1O = new Point3D();
		P1P = new Point3D();
		P2 = new Point3D();
		P3O = new Point3D();
		P3P = new Point3D();
		P4 = new Point3D();
		P13O = new Point3D();
		P43O = new Point3D();
		P21O = new Point3D();
		P13P = new Point3D();
		P43P = new Point3D();
		P21P = new Point3D();
		PaO = new Point3D();
		PbO = new Point3D();
		PmO = new Point3D();
		PaP = new Point3D();
		PbP = new Point3D();
		PmP = new Point3D();
		ndcGaze = new Point3D();
		changeHeadZ = new Point3D();

		ndcGazeO = new Point3D();
		ndcGazeP = new Point3D();
	}

	// View
	public void draw() {
		setCekPengirimanDataKalibrasi(10000);

		background(150);

		if (kalibrasi == 0) {
			// Normalize device coordinates
			stroke(255, 0, 0);
			//line(0, screenResolutionHeight / 2, screenResolutionWidth,
			//		screenResolutionHeight / 2);
			//line(screenResolutionWidth / 2, 0, screenResolutionWidth / 2,
			//		screenResolutionHeight);
			stroke(255);

			// Posisi Pupil berdasarkan metode orthographic
			// P1
			fill(255);
			stroke(0);
			ellipse(ndc2DcP1X, ndc2DcP1Y, 60, 30);

			fill(26, 120, 211);
			ellipse(ndc2DcP1X, ndc2DcP1Y, 20, 20);
			fill(0);
			ellipse(ndc2DcP1X, ndc2DcP1Y, 7, 7);

			fill(255);
			textSize(12);
			text("P1 --> X : " + realWorlKameraKiri.x + " Y : "
					+ realWorlKameraKiri.y, ndc2DcP1X - 200, ndc2DcP1Y + 30);

			// P3
			fill(255);
			stroke(0);
			ellipse(ndc2DcP3X, ndc2DcP3Y, 60, 30);

			fill(26, 120, 211);
			ellipse(ndc2DcP3X, ndc2DcP3Y, 20, 20);
			fill(0);
			ellipse(ndc2DcP3X, ndc2DcP3Y, 7, 7);

			fill(255);
			textSize(12);
			text("P3 --> X : " + realWorlKameraKanan.x + " Y : "
					+ realWorlKameraKanan.y, ndc2DcP3X - 120, ndc2DcP3Y + 30);

			if (isCalibrated) {
				if(eksperimenBoolean){
					// Informasi exmperiment setelah Kalibrasi
					fill(0);
					textSize(12);
					text("Informasi Experiment 3D Gaze ",
							screenResolutionWidth - 366, 30);
					text("Plane :" + plane, screenResolutionWidth - 356, 45);
					text("Point : " + point + " X: " + xPlane + " Y:" + yPlane
							+ " Z:" + zPlane, screenResolutionWidth - 356, 60);
					text("Record Info : " + infoRecord,
							screenResolutionWidth - 356, 75);
					text("Hasil Kalibrasi : " + hasilKalibrasi,
							screenResolutionWidth - 356, 90);

					text("Informasi Keyboard ", screenResolutionWidth - 366, 105);
					text("Tekan Tombol 1-9 Untuk Perubahan Point",
							screenResolutionWidth - 356, 120);
					text("Tekan Tombol Enter Untuk Perubahan Plane",
							screenResolutionWidth - 356, 135);
					text("Tekan Tombol M Untuk Record Data",
							screenResolutionWidth - 356, 150);

					text("Informasi Jarak/Depth ", screenResolutionWidth - 366, 165);
					text("Jarak/Depth Sebelum Kalibrasi: " + depthSebelumKalibrasi
							+ " Cm", screenResolutionWidth - 356, 180);
					text("Jarak/Depth Setelah Kalibrasi: " + jarakUser + " Cm",
							screenResolutionWidth - 356, 195);
					text("Jarak/Depth Plane: " + zPlane + " Cm",
							screenResolutionWidth - 356, 210);

					fill(0, 255, 0);
					noStroke();
					ellipse(ndc2Dc3DGazePX, ndc2Dc3DGazePY, 10, 10);

					// Button Batal Kalibrasi
					fill(0);
					textSize(15);
					text("Clear Kalibrasi", 35, 42);

					fill(0);
					textSize(15);
					text("Record Data", 35, 82);

					// Informasi Perubahan Posisi Kepala terhadap sumbu Z
					if (depthSebelumKalibrasi != jarakUser) {
						changeHeadZ.x = PmO.x * (jarakUser / depthSebelumKalibrasi);
						changeHeadZ.y = PmO.y * (jarakUser / depthSebelumKalibrasi);
						infoChangeHeadZ(changeHeadZ);
						ndc2DcInfoChangeHeadZ(ndcChangeHeadZ);

						// Gaze 3D
						fill(255, 0, 0);
						noStroke();
						ellipse(ndc2Dc3DGazeOXNew, ndc2Dc3DGazeOYNew, 10, 10);
					} else {
						// Gaze 3D
						fill(255, 0, 0);
						noStroke();
						ellipse(ndc2Dc3DGazeOX, ndc2Dc3DGazeOY, 10, 10);
					}

					// Line Of Sight
					stroke(255, 0, 0);
					line(ndc2DcP1X, ndc2DcP1Y, ndc2Dc3DGazePX, ndc2Dc3DGazePY);
					line(ndc2DcP3X, ndc2DcP3Y, ndc2Dc3DGazePX, ndc2Dc3DGazePY);

					// Titik Referensi
					titikReferensi3D();
					// record experiment dan simpan ke file xml
					if (record) {
						durasiRecord++;

						if (durasiRecord <= 10) {

							// String filePath = "F:\\fileExperiment3DGaze1.xml";
							String filePath = "F:/LabTesis/Thesis Baru/HasilExperiment/samsul/3DGazeExperimentResult.xml";
							File xmlFile = new File(filePath);
							DocumentBuilderFactory dbFactory = DocumentBuilderFactory
									.newInstance();
							DocumentBuilder dBuilder;
							try {
								dBuilder = dbFactory.newDocumentBuilder();
								// Document createDoc = dBuilder.newDocument();
								// Element rootElement =
								// createDoc.createElement("Experiment");

								Document doc = dBuilder.parse(xmlFile);
								doc.getDocumentElement().normalize();

								// penerapan elemen anak ke elemen root
								// Formatnya: Document doc, String numPlane, String
								// numPoint, String xPlane, String yPlane, String
								// zPlane, String xGaze, String yGaze, String zGaze
								float xGO = (float) PmO.x;
								float yGO = (float) PmO.y;
								float zGO = (float) PmO.z;

								float xGP = (float) PmP.x;
								float yGP = (float) PmP.y;
								float zGP = (float) PmP.z;

								addElement(doc, Float.toString(jarakUser),
										Integer.toString(point),
										Integer.toString(xPlane),
										Integer.toString(yPlane),
										Integer.toString(zPlane),
										Float.toString(xGO), Float.toString(yGO),
										Float.toString(zGO), Float.toString(xGP),
										Float.toString(yGP), Float.toString(zGP));

								// Simpan file ke dalam bentuk format xml
								doc.getDocumentElement().normalize();
								TransformerFactory transformerFactory = TransformerFactory
										.newInstance();
								Transformer transformer = transformerFactory
										.newTransformer();
								// Buat file xml baru

								// simpan kembali file xml
								DOMSource source = new DOMSource(doc);
								StreamResult result = new StreamResult(
										new File(
												"F:/LabTesis/Thesis Baru/HasilExperiment/samsul/3DGazeExperimentResult.xml"));
								// StreamResult result = new StreamResult(new
								// File("F:\\fileExperiment3DGaze1.xml"));
								transformer.setOutputProperty(OutputKeys.INDENT,
										"yes");
								transformer.transform(source, result);

							} catch (Exception e) {
								// TODO: handle exception
								e.printStackTrace();
							}
						} else {
							record = false;
							infoRecord = "Tidak";
							durasiRecord = 0;
						}
					}
				}else{
					hasilKalibrasi(ndc2Dc3DGazeX, ndc2Dc3DGazeY, hasilKalibrasiAverageErrorDegree);
				}


			} else {
				// Informasi Umum
				fill(0);
				textSize(12);
				text("Informasi Variabel Independent ",
						screenResolutionWidth - 366, 30);
				text("Ukuran IPD Pixel Layar :" + ipdPixelLayar,
						screenResolutionWidth - 356, 45);
				text("Ukuran IPD Real Kamera (mm) : " + ipdCmKameraO * 10,
						screenResolutionWidth - 356, 60);
				text("Jarak User:" + jarakUser, screenResolutionWidth - 356, 75);
				text("Jarak Lurus User:" + jarakLurus,
						screenResolutionWidth - 356, 90);
				text("Tinggi Mata-Monitor:" + tinggiMata,
						screenResolutionWidth - 356, 105);

				text("Informasi Layar Monitor ", screenResolutionWidth - 366,
						120);
				text("Ukuran Fisik (Cm) --> Lebar " + screenPhysicalWidth
						+ " Tinggi : " + screenPhysicalHeight,
						screenResolutionWidth - 356, 135);
				text("Ukuran Resolusi (Pixel) --> Lebar : "
						+ screenResolutionWidth + " Tinggi : "
						+ screenResolutionHeight, screenResolutionWidth - 356,
						150);

				text("Informasi Prosedur Penelitian Ideal",
						screenResolutionWidth - 366, 165);
				text("Tinggi Ideal Antara Eye Tracker - Pengguna: 60 cm",
						screenResolutionWidth - 356, 180);
				text("Jarak Antar Plane: 10 cm", screenResolutionWidth - 356,
						195);
				text("Jarak Antar Point --> X: 12 cm, Y: 6 cm",
						screenResolutionWidth - 356, 210);

				fill(0);
				textSize(24);
				text("Kalibrasi", 39, 45);
			}

			// Button Kalibrasi
			if (mouseX > 25 && mouseY > 20 && mouseX < 150 && mouseY < 55) {

				fill(26, 120, 211);
				stroke(0);
				rect(25, 20, 125, 35, 5);
				rect(-533, -330, -125, -35, 5);

				if (isCalibrated) {
					fill(255);
					textSize(15);
					text("Clear Kalibrasi", -648, -342);
				} else {
					fill(255);
					textSize(24);
					text("Kalibrasi", 39, 45);
				}

			}

			// Button Record Data

			if (mouseX > 25 && mouseY > 60 && mouseX < 150 && mouseY < 95) {
				if (isCalibrated) {
					fill(26, 120, 211);
					stroke(0);
					rect(-533, -280, -125, -35, 5);

					fill(255);
					textSize(15);
					text("Record Data", -648, -302);
				}
			}

		} else {
			if (titikKalibrasi != null) {
				fill(255);
				noStroke();
				float s = map(millis(), mulaiKalibrasi, durasiKalibrasi
						+ mulaiKalibrasi, 40, 20);
				ellipse(titikKalibrasi.x, titikKalibrasi.y, s, s);
				// rect(titikKalibrasi.x, titikKalibrasi.y, s, s);

				fill(0, 0, 0);
				s = map(millis(), mulaiKalibrasi, mulaiKalibrasi
						+ durasiKalibrasi, 1, 7);
				ellipse(titikKalibrasi.x, titikKalibrasi.y, s, s);
				// rect(titikKalibrasi.x, titikKalibrasi.y, s, s);
			}
		}

	}

	// Controller and Model
	public void onGazeUpdate(GazeData gaze, boolean isCalibrated) {
		// Data Gaze
		gazeData = new PVector((float) gaze.smoothedCoordinates.x,
				(float) gaze.smoothedCoordinates.y);

		// Tracking gaze
		if (gazeData != null) {
			tracking.add(gazeData.get());
			if (tracking.size() > 500) {
				tracking.remove(0);
			}
		}

		// Coba Panggil Hasil Gaze
		// System.out.println(gaze.stateToString());
		// System.out.println(jmlHasilKalibrasi);
		// System.out.println(gaze.isFixated);

		// Data Ukuran Pupil
		ukuranPupilKiri = gaze.leftEye.pupilSize.floatValue();
		ukuranPupilKanan = gaze.rightEye.pupilSize.floatValue();

		// Posisi tengah Pupil Setelah Kalibrasi
		rawMataKiri = new PVector((float) gaze.leftEye.smoothedCoordinates.x,
				(float) gaze.leftEye.rawCoordinates.y);
		rawMataKanan = new PVector((float) gaze.rightEye.smoothedCoordinates.x,
				(float) gaze.rightEye.rawCoordinates.y);
		NDCPupilSetelahKalibrasi(rawMataKiri.x, rawMataKiri.y, rawMataKanan.x,
				rawMataKanan.y);
		ndcKalibrasi2Physical(ndcKalibMataKiriX, ndcKalibMataKiriY,
				ndcKalibMataKananX, ndcKalibMataKananY);

		// Data Mata Asli
		normalizeMataKiriX = (float) gaze.leftEye.pupilCenterCoordinates.x;
		normalizeMataKiriY = (float) gaze.leftEye.pupilCenterCoordinates.y;
		normalizeMataKananX = (float) gaze.rightEye.pupilCenterCoordinates.x;
		normalizeMataKananY = (float) gaze.rightEye.pupilCenterCoordinates.y;

		// Data Screen Pixel
		screenMataKiriX = (float) gaze.leftEye.pupilCenterCoordinates.x * width;
		screenMataKiriY = (float) gaze.leftEye.pupilCenterCoordinates.y
				* height;
		screenMataKananX = (float) gaze.rightEye.pupilCenterCoordinates.x
				* width;
		screenMataKananY = (float) gaze.rightEye.pupilCenterCoordinates.y
				* height;

		// NDC
		NDC(screenMataKiriX, screenMataKiriY, screenMataKananX,
				screenMataKananY);
		ndc2Dc(ndcMataKiriX, ndcMataKiriY, ndcMataKananX, ndcMataKananY);
		ndc2Physical(ndcMataKiriX, ndcMataKiriY, ndcMataKananX, ndcMataKananY);
		dataDcNdc(ndcMataKiriX, ndcMataKiriY, ndcMataKananX, ndcMataKananY);

		// Data Screen Pisik dalam satuan Cm
		pisikMataKiriX = (float) gaze.leftEye.pupilCenterCoordinates.x
				* screenPhysicalWidth;
		pisikMataKiriY = (float) gaze.leftEye.pupilCenterCoordinates.y
				* screenPhysicalHeight;
		pisikMataKananX = (float) gaze.rightEye.pupilCenterCoordinates.x
				* screenPhysicalWidth;
		pisikMataKananY = (float) gaze.rightEye.pupilCenterCoordinates.y
				* screenPhysicalHeight;

		// Cek Apakah sudah kalibrasi atau belum
		this.isCalibrated = isCalibrated;

		// inverse transform --> from device coordinate to world coordinate
		dc2W(ndc2DcMataKiriX, ndc2DcMataKiriY, ndc2DcMataKananX,
				ndc2DcMataKananY);

		ipdPixelLayar = hitungIPD(dcNdcMataKiriX, dcNdcMataKiriY,
				dcNdcMataKananX, dcNdcMataKananY);
		jarakUser(ipdPixelLayar);

		// Sudut Pandang Kamera
		pixelKameraKiri.x = gaze.leftEye.pupilCenterCoordinates.x * 300;
		pixelKameraKiri.y = gaze.leftEye.pupilCenterCoordinates.y * 250;
		pixelKameraKanan.x = gaze.rightEye.pupilCenterCoordinates.x * 300;
		pixelKameraKanan.y = gaze.rightEye.pupilCenterCoordinates.y * 250;

		// Tinggi Mata Ke Meja
		tinggiMataKiri2Meja = (screenPhysicalHeight / 2)
				+ ndc2PhysicalMataKiriY;
		tinggiMataKanan2Meja = (screenPhysicalHeight / 2)
				+ ndc2PhysicalMataKananY;

		// Mata dalam model Orthograph proyeksi

		// 3D Mata Kiri Dunia nyata
		P1O.x = realWorlKameraKiri.x;
		P1O.y = realWorlKameraKiri.y;
		P1O.z = 0;

		// 3D Mata Kanan Dunia nyata
		P3O.x = realWorlKameraKanan.x;
		P3O.y = realWorlKameraKanan.y;
		P3O.z = 0;

		// 3D Mata Kiri di layar (fiberglass)
		P2.x = ndcKalib2PhysicalMataKiriX;
		P2.y = ndcKalib2PhysicalMataKiriY;
		P2.z = jarakUser;
		// P2.x = ndc2PhysicalMataKananX;
		// P2.y = ndc2PhysicalMataKananY;
		// P2.z = 0;

		// 3D Mata Kanan di layar (fiberglass)
		P4.x = ndcKalib2PhysicalMataKananX;
		P4.y = ndcKalib2PhysicalMataKananY;
		P4.z = jarakUser;
		// P4.x = ndc2PhysicalMataKiriX;
		// P4.y = ndc2PhysicalMataKiriY;
		// P4.z = 0;

		// Mata dalam model perspektif proyeksi
		// 3D Mata Kiri Dunia Nyata
		P1P.x = ndc2PhysicalMataKiriX;
		P1P.y = ndc2PhysicalMataKiriY;
		P1P.z = 0;

		// 3D Mata Kanan Dunia nyata
		P3P.x = ndc2PhysicalMataKananX;
		P3P.y = ndc2PhysicalMataKananY;
		P3P.z = 0;

		// Intersection Line of Sight Othographi
		P13O = P1O.subtract(P3O);
		P43O = P4.subtract(P3O);
		P21O = P2.subtract(P1O);

		// Intersection Line of Sight Proyeksi
		P13P = P1P.subtract(P3P);
		P43P = P4.subtract(P3P);
		P21P = P2.subtract(P1P);

		double d1343P = P13P.x * (double) P43P.x + (double) P13P.y * P43P.y
				+ (double) P13P.z * P43P.z;
		double d4321P = P43P.x * (double) P21P.x + (double) P43P.y * P21P.y
				+ (double) P43P.z * P21P.z;
		double d1321P = P13P.x * (double) P21P.x + (double) P13P.y * P21P.y
				+ (double) P13P.z * P21P.z;
		double d4343P = P43P.x * (double) P43P.x + (double) P43P.y * P43P.y
				+ (double) P43P.z * P43P.z;
		double d2121P = P21P.x * (double) P21P.x + (double) P21P.y * P21P.y
				+ (double) P21P.z * P21P.z;

		double denomP = d2121P * d4343P - d4321P * d4321P;
		double numerP = d1343P * d4321P - d1321P * d4343P;
		double muaP = numerP / denomP;
		double mubP = (d1343P + d4321P * (muaP)) / d4343P;

		double d1343O = P13O.x * (double) P43O.x + (double) P13O.y * P43O.y
				+ (double) P13O.z * P43O.z;
		double d4321O = P43O.x * (double) P21O.x + (double) P43O.y * P21O.y
				+ (double) P43O.z * P21O.z;
		double d1321O = P13O.x * (double) P21O.x + (double) P13O.y * P21O.y
				+ (double) P13O.z * P21O.z;
		double d4343O = P43O.x * (double) P43O.x + (double) P43O.y * P43O.y
				+ (double) P43O.z * P43O.z;
		double d2121O = P21O.x * (double) P21O.x + (double) P21O.y * P21O.y
				+ (double) P21O.z * P21O.z;

		double denomO = d2121O * d4343O - d4321O * d4321O;
		double numerO = d1343O * d4321O - d1321O * d4343O;
		double muaO = numerO / denomO;
		double mubO = (d1343O + d4321O * (muaO)) / d4343O;

		PaP.x = (float) (P1P.x + muaP * P21P.x);
		PaP.y = (float) (P1P.y + muaP * P21P.y);
		PaP.z = (float) (P1P.z + muaP * P21P.z);
		PbP.x = (float) (P3P.x + mubP * P43P.x);
		PbP.y = (float) (P3P.y + mubP * P43P.y);
		PbP.z = (float) (P3P.z + mubP * P43P.z);

		PaO.x = (float) (P1O.x + muaO * P21O.x);
		PaO.y = (float) (P1O.y + muaO * P21O.y);
		PaO.z = (float) (P1O.z + muaO * P21O.z);
		PbO.x = (float) (P3O.x + mubO * P43O.x);
		PbO.y = (float) (P3O.y + mubO * P43O.y);
		PbO.z = (float) (P3O.z + mubO * P43O.z);

		PmP = PaP.add(PbP);
		PmP = PmP.divide(2);

		PmO = PaO.add(PbO);
		PmO = PmO.divide(2);

		// 3D gaze data ditampilkan di graphics/screen
		w2Ndc3DGazeO(PmO);
		w2Ndc3DGazeP(PmP);
		ndc2Dc3DGazeO(ndcGazeO);
		ndc2Dc3DGazeP(ndcGazeP);

		// Data P1 ditampilkan di graphics/screen
		w2NdcP1(P1O);
		ndc2DcP1(ndcP1);

		// Data P2 ditampilkan di graphics/screen
		w2NdcP3(P3O);
		ndc2DcP3(ndcP3);
		
		//pandangan 2D untuk tes hasil kalibrasi
		w2Ndc3DGaze(PmO);
		ndc2Dc3DGaze(ndcGaze);
		
		// Nilai z Disparity / Vergence
		zDisparity = zDisparity(P2, P4, ipdCmKameraO, jarakUser);

		// Jarak versi 2
		float y = (float) (P1O.y + (screenPhysicalHeight / 2));
		tinggiMata = y;
		jarakLurus = (float) Math.sqrt(Math.pow(jarakUser, 2) - Math.pow(y, 2));

		// NDCKamera(pixelKameraKiri, pixelKameraKanan);
		// ndc2Kamera(ndcKameraMataKiri, ndcKameraMataKanan);
		mapping3DToRealWorld(pixelKameraKiri, pixelKameraKanan);

		// IPD Kamera
		ipdCmKameraO = hitungIPD(realWorlKameraKiri.x, realWorlKameraKiri.y,
				realWorlKameraKanan.x, realWorlKameraKanan.y);
		ipdCmKameraP = hitungIPD(P1P.x, P1P.y, P3P.x, P3P.y);

		ipdPixelO = hitungIPD(ndc2DcP1X, ndc2DcP1Y, ndc2DcP3X, ndc2DcP3Y);
		ipdPixelP = hitungIPD(ndc2DcMataKiriX, ndc2DcMataKiriY,
				ndc2DcMataKananX, ndc2DcMataKananX);

	}

	public void screenResolution(float screenPhysicalHeight_,
			float screenPhysicalWidth_, float screenResolutionHeight_,
			float screenResolutionWidth_) {
		screenPhysicalHeight = screenPhysicalHeight_;
		screenPhysicalWidth = screenPhysicalWidth_;
		screenResolutionHeight = screenResolutionHeight_;
		screenResolutionWidth = screenResolutionWidth_;
	}

	public void titikKalibrasi(PVector p, boolean mulai) {
		// TODO Auto-generated method stub
		if (mulai) {
			// println("Titik Kalibrasi: " + p);
			titikKalibrasi = p.get();
			mulaiKalibrasi = millis();
		} else {
			// println("Selesai dengan " + p);
			titikKalibrasi = null;
		}
	}

	public void akhirDariKalibrasi(CalibrationResult hasil) {
		kalibrasi = 0;
		
		// println("Hasil: " + hasil.averageErrorDegree);
		hasilKalibrasiAverageErrorDegree = hasil.averageErrorDegree;
		if (hasil.result.booleanValue()) {
			hasilKalibrasi = akurasi(hasil.averageErrorDegree);
		} else {
			hasilKalibrasi = "Kalibrasi gagal, ulangi lagi";
		}
	}

	private String akurasi(double hasil) {
		if (hasil < 0.5) {
			return "Sempurna";
		}
		if (hasil < 0.7) {
			return "Bagus";
		}
		if (hasil < 1) {
			return "Sedang";
		}
		if (hasil < 1.5) {
			return "Buruk";
		}
		return "Ulangi Lagi";
	}

	public void mouseClicked() {
		// TODO Auto-generated method stub
		super.mouseClicked();

		// Tombol button untuk kalibrasi dan clear kalibrasi
		if (mouseX > 25 && mouseY > 20 && mouseX < 150 && mouseY < 55) {
			if (isCalibrated) {				
				eyeTribe.clearKalibrasi();
				eksperimenBoolean = false;
			} else {
				eyeTribe.gantiHasilKalibrasi(calibrationResult1);

				kalibrasi++;
				calibResult++;

				ArrayList<PVector> titikTitik = new ArrayList<PVector>();

				titikTitik.add(new PVector(144, 116));
				titikTitik.add(new PVector(680, 116));
				titikTitik.add(new PVector(1224, 116));

				titikTitik.add(new PVector(144, 383));
				titikTitik.add(new PVector(680, 383));
				titikTitik.add(new PVector(1224, 383));

				titikTitik.add(new PVector(144, 653));
				titikTitik.add(new PVector(680, 653));
				titikTitik.add(new PVector(1224, 653));

				PVector panggilPoint[] = titikTitik
						.toArray(new PVector[titikTitik.size()]);
				eyeTribe.kalibrasi(panggilPoint);

				zPlane = (int) jarakUser;
				depthSebelumKalibrasi = (int) jarakUser;
			}
		}
		// Tombol button untuk record data
		if (mouseX > 25 && mouseY > 60 && mouseX < 150 && mouseY < 95) {
			record = true;
			infoRecord = "Ya";
		}
	}

	public float hitungIPD(double x, double y, double x2, double y2) {
		float ipd;
		ipd = (float) Math.sqrt(Math.pow((x2 - x), 2) + Math.pow((y2 - y), 2));
		return ipd;
	}

	// Mapping koordinat dari "top left" ke "center of screen"
	public void NDC(float pixelMataKiriX, float pixelMataKiriY,
			float pixelMataKananX, float pixelMataKananY) {
		ndcMataKiriX = (float) ((pixelMataKiriX * 2) / width) - 1;
		ndcMataKananX = (float) ((pixelMataKananX * 2) / width) - 1;
		ndcMataKiriY = (float) (1 - (pixelMataKiriY * 2) / height);
		ndcMataKananY = (float) (1 - (pixelMataKananY * 2) / height);
	}

	public void NDCPupilSetelahKalibrasi(float pixelKalibMataKiriX,
			float pixelKalibMataKiriY, float pixelKalibMataKananX,
			float pixelKalibMataKananY) {
		ndcKalibMataKiriX = (float) ((pixelKalibMataKiriX * 2) / width) - 1;
		ndcKalibMataKananX = (float) ((pixelKalibMataKananX * 2) / width) - 1;
		ndcKalibMataKiriY = (float) (1 - (pixelKalibMataKiriY * 2) / height);
		ndcKalibMataKananY = (float) (1 - (pixelKalibMataKananY * 2) / height);
	}

	// Konver data pixel eye tribe ke format ndc
	public void ndc2Dc(float ndcMataKiriX, float ndcMataKiriY,
			float ndcMataKananX, float ndcMataKananY) {
		ndc2DcMataKiriX = ((ndcMataKiriX * (width / 2))) + width / 2;
		ndc2DcMataKiriY = (height / 2 - (ndcMataKiriY * (height / 2)));
		ndc2DcMataKananX = ((ndcMataKananX * (width / 2))) + width / 2;
		ndc2DcMataKananY = (height / 2 - (ndcMataKananY * (height / 2)));
	}

	// Mendapatkan data mata fisik satuan cm yang dihitung dari tengah layar
	// Baik itu x maupun y
	public void ndc2Physical(float ndcMataKiriX, float ndcMataKiriY,
			float ndcMataKananX, float ndcMataKananY) {
		ndc2PhysicalMataKiriX = ndcMataKiriX * (screenPhysicalWidth / 2);
		ndc2PhysicalMataKiriY = ndcMataKiriY * (screenPhysicalHeight / 2);
		ndc2PhysicalMataKananX = ndcMataKananX * (screenPhysicalWidth / 2);
		ndc2PhysicalMataKananY = ndcMataKananY * (screenPhysicalHeight / 2);
	}

	public void ndcKalibrasi2Physical(float ndcKalibMataKiriX,
			float ndcKalibMataKiriY, float ndcKalibMataKananX,
			float ndcKalibMataKananY) {
		ndcKalib2PhysicalMataKiriX = ndcKalibMataKiriX
				* (screenPhysicalWidth / 2);
		ndcKalib2PhysicalMataKiriY = ndcKalibMataKiriY
				* (screenPhysicalHeight / 2);
		ndcKalib2PhysicalMataKananX = ndcKalibMataKananX
				* (screenPhysicalWidth / 2);
		ndcKalib2PhysicalMataKananY = ndcKalibMataKananY
				* (screenPhysicalHeight / 2);
	}

	// Mendapatkan data mata satuan pixel yang dihitung dari tengah layar
	// Baik itu x maupun y
	private void dataDcNdc(float ndcMataKiriX, float ndcMataKiriY,
			float ndcMataKananX, float ndcMataKananY) {
		// TODO Auto-generated method stub
		dcNdcMataKiriX = ndcMataKiriX * (screenResolutionWidth / 2);
		dcNdcMataKiriY = ndcMataKiriY * (screenResolutionHeight / 2);
		dcNdcMataKananX = ndcMataKananX * (screenResolutionWidth / 2);
		dcNdcMataKananY = ndcMataKananY * (screenResolutionHeight / 2);
	}

	// Konersi data ndc ke pixel dengan titik (0,0) berada di "top left"
	public void w2Ndc(float wMataKiriX, float wMataKiriY, float wMataKananX,
			float wMataKananY) {
		ndc2DcMataKiriX = ((ndcMataKiriX * width / 2)) + width / 2;
		ndc2DcMataKiriY = ((ndcMataKiriY * height / 2)) + height / 2;
		ndc2DcMataKananX = ((ndcMataKananX * width / 2)) + width / 2;
		ndc2DcMataKananY = ((ndcMataKananY * height / 2)) + height / 2;
	}

	// Mendapatkan P1 pixel dari world (fisik) coordinat
	public void w2NdcP1(Point3D P1) {
		ndcP1.x = (float) P1.x / (screenPhysicalWidth / 2);
		ndcP1.y = (float) P1.y / (screenPhysicalHeight / 2);
	}

	public void ndc2DcP1(Point2D ndcP1) {
		ndc2DcP1X = (float) (((ndcP1.x * (width / 2))) + width / 2);
		ndc2DcP1Y = (float) (height / 2 - (ndcP1.y * (height / 2)));
	}

	// Mendapatkan P3 pixel dari world (fisik) coordinat
	public void w2NdcP3(Point3D P3) {
		ndcP3.x = (float) P3.x / (screenPhysicalWidth / 2);
		ndcP3.y = (float) P3.y / (screenPhysicalHeight / 2);
	}

	public void ndc2DcP3(Point2D ndcP3) {
		ndc2DcP3X = (float) (((ndcP3.x * (width / 2))) + width / 2);
		ndc2DcP3Y = (float) (height / 2 - (ndcP3.y * (height / 2)));
	}

	// Mendapatkan Gaze pixel dari world (fisik) coordinat
	public void w2Ndc3DGazeO(Point3D gazeWorld3D) {
		ndcGazeO.x = (float) gazeWorld3D.x / (screenPhysicalWidth / 2);
		ndcGazeO.y = (float) gazeWorld3D.y / (screenPhysicalHeight / 2);
	}

	// Mendapatkan Gaze pixel dari world (fisik) coordinat
	public void w2Ndc3DGazeP(Point3D gazeWorld3D) {
		ndcGazeP.x = (float) gazeWorld3D.x / (screenPhysicalWidth / 2);
		ndcGazeP.y = (float) gazeWorld3D.y / (screenPhysicalHeight / 2);
	}

	// Informasi Perubahan Posisi Kepala terhadap sumbu Z
	public void infoChangeHeadZ(Point3D changeHeadZ) {
		ndcChangeHeadZ.x = (float) changeHeadZ.x / (screenPhysicalWidth / 2);
		ndcChangeHeadZ.y = (float) changeHeadZ.y / (screenPhysicalHeight / 2);
	}

	public void ndc2DcInfoChangeHeadZ(Point2D ndcChangeHeadZ) {
		ndc2Dc3DGazeOXNew = (float) (((ndcChangeHeadZ.x * (width / 2))) + width / 2);
		ndc2Dc3DGazeOYNew = (float) (height / 2 - (ndcChangeHeadZ.y * (height / 2)));
	}
	public void ndc2Dc3DGaze(Point3D gaze3D) {
		ndc2Dc3DGazeX = (float) (((gaze3D.x * (width / 2))) + width / 2);
		ndc2Dc3DGazeY = (float) (height / 2 - (gaze3D.y * (height / 2)));
	}
	
	// Informasi Perubahan Posisi Kepala terhadap sumbu X dan Y
	public void infoChangeHeadXY(float initialX, float initialY, float nextX,
			float nextY) {

	}

	// mendapatkan nilai/sudut vergence dan disparity mata untuk melihat
	// perubahan z
	public double zDisparity(Point3D mataKiri, Point3D mataKanan, double ipd,
			double jarak) {
		double z;
		double deltaX = mataKanan.x - mataKiri.x;
		double D = Math.sqrt(Math.pow(jarak, 2)
				+ Math.pow(((mataKiri.y + mataKanan.y) / 2), 2));

		z = (deltaX * D) / deltaX - ipd;
		return z;
	}

	public void ndc2Dc3DGazeO(Point3D gaze3D) {
		ndc2Dc3DGazeOX = (float) (((gaze3D.x * (width / 2))) + width / 2);
		ndc2Dc3DGazeOY = (float) (height / 2 - (gaze3D.y * (height / 2)));
	}

	public void ndc2Dc3DGazeP(Point3D gaze3D) {
		ndc2Dc3DGazePX = (float) (((gaze3D.x * (width / 2))) + width / 2);
		ndc2Dc3DGazePY = (float) (height / 2 - (gaze3D.y * (height / 2)));
	}
	
	public void w2Ndc3DGaze(Point3D gazeWorld3D) {
		ndcGaze.x = (float) gazeWorld3D.x / (screenPhysicalWidth / 2);
		ndcGaze.y = (float) gazeWorld3D.y / (screenPhysicalHeight / 2);
	}
	// Jarak User ke sistem
	public void jarakUser(float ipdBaru) {
		jarakUser = (float) ((263 * 58) / ipdBaru);
	}

	// inverse transform --> from device coordinate to world coordinate
	public void dc2W(float dcMataKiriX, float dcMataKiriY, float dcMataKananX,
			float dcMataKananY) {
		wMataKiriX = ((dcMataKiriX - (screenResolutionWidth / 2))
				* screenPhysicalWidth / screenResolutionWidth)
				+ screenPhysicalWidth / 2;
		wMataKiriY = ((dcMataKiriY - (screenResolutionHeight / 2))
				* screenPhysicalHeight / screenResolutionHeight)
				+ screenPhysicalHeight / 2;
		wMataKananX = ((dcMataKananX - (screenResolutionWidth / 2))
				* screenPhysicalWidth / screenResolutionWidth)
				+ screenPhysicalWidth / 2;
		wMataKananY = ((dcMataKananY - (screenResolutionHeight / 2))
				* screenPhysicalHeight / screenResolutionHeight)
				+ screenPhysicalHeight / 2;
	}

	// sudut visual angle
	public void sudutVisualAngle(float ipd, float jarak) {
		float oppt = ipd / 2;
		float tan = oppt / jarak;
		sudut = (float) Math.toDegrees(Math.atan(tan));
	}

	// jarak user sesuai rumus trigonometri
	public void jarakUserTrigonometri(float ipd) {
		float oppt = ipd / 2;
		jarakUserTrigonometri = (float) (oppt / Math.tan(Math.toRadians(2)));
	}

	// Mendapatakan ukuran mata dari kamera
	// Resolusi--> W:300 H:250
	private void trackBoxCamera(float ndcMataKiriX, float ndcMataKiriY,
			float ndcMataKananX, float ndcMataKananY) {
		float dx = ndcMataKananX - ndcMataKiriX;
		float dy = ndcMataKananY - ndcMataKiriY;

		latestNormalizeDistance = (float) Math.sqrt(dx * dx + dy * dy);
		latestAngle = (float) (RadToDeg * Math.atan2(dy * 250, dx * 300));
	}

	private double doEyeSizeDiff() {
		// TODO Auto-generated method stub
		double b = 0.15;
		double a = 1;
		return ((latestNormalizeDistance - b) / b) * a;
	}

	// Konver pixel ke NDC sudut pandang Kamera
	public void NDCKamera(Point2D pixelMataKiri, Point2D pixelMataKanan) {
		ndcKameraMataKiri.x = (float) ((pixelMataKiri.x * 2) / 300) - 1;
		ndcKameraMataKanan.x = (float) ((pixelMataKanan.x * 2) / 300) - 1;
		ndcKameraMataKiri.y = (float) (1 - (pixelMataKiri.y * 2) / 150);
		ndcKameraMataKanan.y = (float) (1 - (pixelMataKanan.y * 2) / 150);
	}

	private void ndc2Kamera(Point2D ndcKameraMataKiri,
			Point2D ndcKameraMataKanan) {
		kameraNdcMataKiri.x = ndcKameraMataKiri.x * (300 / 2);
		kameraNdcMataKiri.y = ndcKameraMataKiri.y * (250 / 2);
		kameraNdcMataKanan.x = ndcKameraMataKanan.x * (300 / 2);
		kameraNdcMataKanan.y = ndcKameraMataKanan.y * (250 / 2);
	}

	// Mendapatkan data real world Kamera --> P1 dan P3
	private void mapping3DToRealWorld(Point2D pixelMataKiri,
			Point2D pixelMataKanan) {
		realWorlKameraKanan.x = 2 * (pixelMataKanan.x - (299 / 2))
				* Math.tan(Math.toRadians(33 / 2)) * jarakUser / 300;
		realWorlKameraKiri.x = 2 * (pixelMataKiri.x - (299 / 2))
				* Math.tan(Math.toRadians(33 / 2)) * jarakUser / 300;
		realWorlKameraKanan.y = 2 * (249 - pixelMataKanan.y - (249 / 2))
				* Math.tan(Math.toRadians(33 / 2)) * jarakUser / 250;
		realWorlKameraKiri.y = 2 * (249 - pixelMataKiri.y - (249 / 2))
				* Math.tan(Math.toRadians(33 / 2)) * jarakUser / 250;
	}

	// Angular horizontal

	// Angual Vertical

	// Simpan file dalam bentuk format xml
	private static Node getPlane(Document doc, String numPlane,
			String numPoint, String xPlane, String yPlane, String zPlane,
			String xGazeO, String yGazeO, String zGazeO, String xGazeP,
			String yGazeP, String zGazeP) {
		Element plane = doc.createElement("Plane");
		plane.setAttribute("numberPlane", numPlane);
		plane.appendChild(getPlaneElement(doc, plane, "numberPoint", numPoint));
		plane.appendChild(getPlaneElement(doc, plane, "xPlane", xPlane));
		plane.appendChild(getPlaneElement(doc, plane, "yPlane", yPlane));
		plane.appendChild(getPlaneElement(doc, plane, "zPlane", zPlane));

		plane.appendChild(getPlaneElement(doc, plane, "xGazeO", xGazeO));
		plane.appendChild(getPlaneElement(doc, plane, "yGazeO", yGazeO));
		plane.appendChild(getPlaneElement(doc, plane, "zGazeO", zGazeO));

		plane.appendChild(getPlaneElement(doc, plane, "xGazeP", xGazeP));
		plane.appendChild(getPlaneElement(doc, plane, "yGazeP", yGazeP));
		plane.appendChild(getPlaneElement(doc, plane, "zGazeP", zGazeP));

		return plane;
	}

	private static void addElement(Document doc, String numPlane,
			String numPoint, String xPlane, String yPlane, String zPlane,
			String xGazeO, String yGazeO, String zGazeO, String xGazeP,
			String yGazeP, String zGazeP) {
		NodeList experiment = doc.getElementsByTagName("Experiment");
		Element emp = null;

		//
		for (int i = 0; i < experiment.getLength(); i++) {
			emp = (Element) experiment.item(i);
			Element plane = doc.createElement("Plane");
			plane.setAttribute("numberPlane", numPlane);
			plane.appendChild(getPlaneElement(doc, plane, "numberPoint",
					numPoint));
			plane.appendChild(getPlaneElement(doc, plane, "xPlane", xPlane));
			plane.appendChild(getPlaneElement(doc, plane, "yPlane", yPlane));
			plane.appendChild(getPlaneElement(doc, plane, "zPlane", zPlane));

			plane.appendChild(getPlaneElement(doc, plane, "xGazeO", xGazeO));
			plane.appendChild(getPlaneElement(doc, plane, "yGazeO", yGazeO));
			plane.appendChild(getPlaneElement(doc, plane, "zGazeO", zGazeO));

			plane.appendChild(getPlaneElement(doc, plane, "xGazeP", xGazeP));
			plane.appendChild(getPlaneElement(doc, plane, "yGazeP", yGazeP));
			plane.appendChild(getPlaneElement(doc, plane, "zGazeP", zGazeP));
			emp.appendChild(plane);
		}
	}

	private static Node getPlaneElement(Document doc, Element element,
			String name, String value) {
		Element node = doc.createElement(name);
		node.appendChild(doc.createTextNode(value));
		return node;
	}

	public void mousePressed() {
		// save("f:/gaze3D2TinggiMata.jpg");
		// background(0);
		// record = true;
		// infoRecord = "Ya";
	}

	@Override
	public void keyPressed() {
		// TODO Auto-generated method stub
		// super.keyPressed();

		if (key == ENTER) {
			plane++;
			zPlane = zPlane - 10;

			calibrationResult = calibrationResult1;
			setCekPengirimanDataKalibrasi(40);

			eyeTribe.gantiHasilKalibrasi(calibrationResult1);
		}
		if (key == '1') {
			point = 1;

			xPlane = -12;
			yPlane = -6;
		}
		if (key == '2') {
			point = 2;

			xPlane = 0;
			yPlane = -6;
		}
		if (key == '3') {
			point = 3;

			xPlane = 12;
			yPlane = -6;
		}
		if (key == '4') {
			point = 4;

			xPlane = -12;
			yPlane = 0;
		}
		if (key == '5') {
			point = 5;

			xPlane = 0;
			yPlane = 0;
		}
		if (key == '6') {
			point = 6;

			xPlane = 12;
			yPlane = 0;
		}
		if (key == '7') {
			point = 7;

			xPlane = -12;
			yPlane = 6;
		}
		if (key == '8') {
			point = 8;

			xPlane = 0;
			yPlane = 6;
		}
		if (key == '9') {
			point = 9;

			xPlane = 12;
			yPlane = 6;
		}
		// record experiment
		if (key == 'm') {
			record = true;
			infoRecord = "Ya";
		}
		if (key == 'c') {
			record = false;
			infoRecord = "Tidak";
		}
		if (key == 'e') {
			eksperimenBoolean = true;
		}

		// Print Screen Citra
		if (key == 'p') {
			save("F:/LabTesis/Thesis Baru/PrintScreenImage/gaze3dnew.jpg");
		}
	}

	// Switch Hasil Kalibrasi
	public void calibrationSwitch(int hasil) {
		System.out.println(hasil);

		/*
		 * ICalibrationResultListener listener = new
		 * ICalibrationResultListener() {
		 * 
		 * public void onCalibrationChanged(boolean isCalibrated,
		 * CalibrationResult calibResult) { // TODO Auto-generated method stub
		 * isCalibrated = true; calibResult = calibrationResult; } };
		 * System.out.println("Berjalan");
		 * 
		 * GazeManager gazeManager = GazeManager.getInstance(); boolean sukses =
		 * gazeManager.activate(GazeManager.ApiVersion.VERSION_1_0,
		 * GazeManager.ClientMode.PUSH);
		 * 
		 * gazeManager.addCalibrationResultListener(listener);
		 */
	}

	public void titikReferensi3D() {
		stroke(255);
		noFill();
		translate(width / 2, (height / 2)); // titik di 0,0 ditengah2 layar

		// Garis pinggir
		line(-600, 84, -600, 384);
		line(-200, 4, -200, -80);
		line(200, 4, 200, -80);
		line(600, 84, 600, 384);
		line(-600, 84, -200, -80);
		line(600, 84, 200, -80);
		line(-200, -80, 200, -80);

		line(0, 3, 0, 384);
		line(-200, 3, 200, 3);
		// Garis Bagian Kanan
		line(50, 3, 150, 384);
		line(100, 3, 300, 384);
		line(150, 3, 450, 384);
		line(200, 3, 600, 384);

		// Garis Bagian Kiri
		line(-50, 3, -150, 384);
		line(-100, 3, -300, 384);
		line(-150, 3, -450, 384);
		line(-200, 3, -600, 384);

		// Gari Potong
		line(-230, 30, 230, 30);
		line(-275, 75, 275, 75);
		line(-355, 150, 355, 150);
		line(-460, 250, 460, 250);

		// Posisi titik referensi bagian kiri
		fill(0, 0, 255);
		stroke(0);
		rect(-60, -7, 10, 10);
		rect(-160, 55, 20, 20);
		rect(-270, 210, 40, 40);

		// Posisi titik referensi bagian tengah
		rect(0, -7, 10, 10);
		rect(0, 55, 20, 20);
		rect(0, 210, 40, 40);

		// Posisi titik referensi bagian kanan
		rect(50, -7, 10, 10);
		rect(140, 55, 20, 20);
		rect(230, 210, 40, 40);
	}

	public void hasilKalibrasi(float ndc2Dc3DGazeX, float ndc2Dc3DGazeY,
			double hasilKalibrasiAverageErrorDegree) {
		
		// Button Batal Kalibrasi
		fill(0);
		textSize(15);
		text("Clear Kalibrasi", 35, 42);
		
		// Pengujian hasil kalibrasi
		fill(255);
		// Posisi Fisik (cm) --> X: -12 Y: 6
		ellipse(144, 116, 30, 30);
		fill(0);
		textSize(14);
		text("1", 144 - 4, 116 + 5);
		// Posisi Fisik (cm) --> X: 0 Y: 6
		fill(255);
		ellipse(680, 116, 30, 30);
		fill(0);
		textSize(14);
		text("2", 680 - 4, 116 + 5);
		// Posisi Fisik (cm) --> X: 12 Y: 6
		fill(255);
		ellipse(1224, 116, 30, 30);
		fill(0);
		textSize(14);
		text("3", 1224 - 4, 116 + 5);
		// Posisi Fisik (cm) --> X: -12 Y: 0
		fill(255);
		ellipse(144, 383, 30, 30);
		fill(0);
		textSize(14);
		text("4", 144 - 4, 383 + 5);
		// Posisi Fisik (cm) --> X: -0 Y: 0
		fill(255);
		ellipse(680, 383, 30, 30);
		fill(0);
		textSize(14);
		text("5", 680 - 4, 383 + 5);
		// Posisi Fisik (cm) --> X: 12 Y: 0
		fill(255);
		ellipse(1224, 383, 30, 30);
		fill(0);
		textSize(14);
		text("6", 1224 - 4, 383 + 5);
		// Posisi Fisik (cm) --> X: -12 Y: -6
		fill(255);
		ellipse(144, 653, 30, 30);
		fill(0);
		textSize(14);
		text("7", 144 - 4, 653 + 5);
		// Posisi Fisik (cm) --> X: 0 Y: -6
		fill(255);
		ellipse(680, 653, 30, 30);
		fill(0);
		textSize(14);
		text("8", 680 - 4, 653 + 5);
		// Posisi Fisik (cm) --> X: 12 Y: -6
		fill(255);
		ellipse(1224, 653, 30, 30);
		fill(0);
		textSize(14);
		text("9", 1224 - 4, 653 + 5);

		if ((ndc2Dc3DGazeX >= 144) && (ndc2Dc3DGazeX <= 144 + 50)
				&& (ndc2Dc3DGazeY >= 116) && (ndc2Dc3DGazeY <= 116 + 50)) {
			fill(0, 0, 255);
			ellipse(144, 116, 30, 30);
			fill(255);
			textSize(14);
			text("1", 144 - 4, 116 + 5);
		}
		if ((ndc2Dc3DGazeX >= 680) && (ndc2Dc3DGazeX <= 680 + 50)
				&& (ndc2Dc3DGazeY >= 116) && (ndc2Dc3DGazeY <= 116 + 50)) {
			fill(0, 0, 255);
			ellipse(680, 116, 30, 30);
			fill(255);
			textSize(14);
			text("2", 680 - 4, 116 + 5);
		}
		if ((ndc2Dc3DGazeX >= 1224) && (ndc2Dc3DGazeX <= 1224 + 50)
				&& (ndc2Dc3DGazeY >= 116) && (ndc2Dc3DGazeY <= 116 + 50)) {
			fill(0, 0, 255);
			ellipse(1224, 116, 30, 30);
			fill(255);
			textSize(14);
			text("3", 1224 - 4, 116 + 5);
		}

		if ((ndc2Dc3DGazeX >= 144) && (ndc2Dc3DGazeX <= 144 + 50)
				&& (ndc2Dc3DGazeY >= 383) && (ndc2Dc3DGazeY <= 383 + 50)) {
			fill(0, 0, 255);
			ellipse(144, 383, 30, 30);
			fill(255);
			textSize(14);
			text("4", 144 - 4, 383 + 5);

		}
		if ((ndc2Dc3DGazeX >= 680) && (ndc2Dc3DGazeX <= 680 + 50)
				&& (ndc2Dc3DGazeY >= 383) && (ndc2Dc3DGazeY <= 383 + 50)) {
			fill(0, 0, 255);
			ellipse(680, 383, 30, 30);
			fill(255);
			textSize(14);
			text("5", 680 - 4, 383 + 5);
		}
		if ((ndc2Dc3DGazeX >= 1224) && (ndc2Dc3DGazeX <= 1224 + 50)
				&& (ndc2Dc3DGazeY >= 383) && (ndc2Dc3DGazeY <= 383 + 50)) {
			fill(0, 0, 255);
			ellipse(1224, 383, 30, 30);
			fill(255);
			textSize(14);
			text("6", 1224 - 4, 383 + 5);
		}

		if ((ndc2Dc3DGazeX >= 144) && (ndc2Dc3DGazeX <= 144 + 50)
				&& (ndc2Dc3DGazeY >= 653) && (ndc2Dc3DGazeY <= 653 + 50)) {
			fill(0, 0, 255);
			ellipse(144, 653, 30, 30);
			fill(255);
			textSize(14);
			text("7", 144 - 4, 653 + 5);
		}
		if ((ndc2Dc3DGazeX >= 680) && (ndc2Dc3DGazeX <= 680 + 50)
				&& (ndc2Dc3DGazeY >= 653) && (ndc2Dc3DGazeY <= 653 + 50)) {
			fill(0, 0, 255);
			ellipse(680, 653, 30, 30);
			fill(255);
			textSize(14);
			text("8", 680 - 4, 653 + 5);
		}
		if ((ndc2Dc3DGazeX >= 1224) && (ndc2Dc3DGazeX <= 1224 + 50)
				&& (ndc2Dc3DGazeY >= 653) && (ndc2Dc3DGazeY <= 653 + 50)) {
			fill(0, 0, 255);
			ellipse(1224, 653, 30, 30);
			fill(255);
			textSize(14);
			text("9", 1224 - 4, 653 + 5);
		}

		// Informasi hasil kalibrasi
		fill(255);
		textSize(16);
		text("Hasil Kalibrasi: " + hasilKalibrasi
				+ ", Average Error Kalibrasi:  "
				+ hasilKalibrasiAverageErrorDegree +", Tekan Tombol E Untuk Mulai Eksperimen",
				screenResolutionWidth / 2 - 350, screenResolutionHeight - 50);

	}

	public static void main(String[] args) {
		PApplet.main(new String[] { main.MataProduction.class.getName() });
	}

}
