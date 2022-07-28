using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using UnityEngine; //TODO TO remove

using CORC;
using trakSTAR;


namespace MvtLogging
{
	public class MvtLogger
	{
		private bool readyToRecord = false;
		private bool recording = false;
		private bool recordingRunning = false;
		private Thread recordingThread = null;
		private CORCM3_FOURIER robot;
		private FLNLClient mediaPipe;
		private trakSTARSensors trakstar;
		private int nbTrakstarSensors = 3;
		
		public StreamWriter logFileStream;
		private string fileHeader = "#t (abs), t, X_1, X_2, X_3, dX_1, dX_2, dX_3, F_1, F_2, F_3, Cmd, MvtProg, Contrib, S1_x, S1_y, S1_z, S1_a, S1_e, S1_r, S2_x, S2_y, S2_z, S2_a, S2_e, S2_r, S3_x, S3_y, S3_z, S3_a, S3_e, S3_r\n"; //Ooohhh ! this is rigid and ugly!
		
		public MvtLogger(CORCM3_FOURIER r)
		{
			//M3 robot
			robot = r;
			
			//Connection to local Dept camera python script
			mediaPipe = new FLNLClient();
			
			//trakSTAR
			trakstar = new trakSTARSensors();
		}
		
		public bool InitSensors()
		{
			Stop();
			
			if(mediaPipe.Connect("127.0.0.1", 2042)==false)
				return false;

			return trakstar.Init();
		}
		
		~MvtLogger()
		{
			Stop();
		}
		
		public bool Init(string filename)
		{
			//Init trakSTAR
			if(mediaPipe.IsConnected() && trakstar.IsInitialised() && robot.IsInitialised())
			{
				//Create file and header
				logFileStream = new StreamWriter(filename);
				logFileStream.Write(fileHeader);
				
				readyToRecord = true;
			}
			else
			{
				readyToRecord = false;
			}
			return readyToRecord;
		}
		
		public bool Start()
		{
			if(readyToRecord)
			{
				recording = true;
				recordingRunning = true;
				
				//Start MediaPipe stream
				mediaPipe.SendCmd("STA".ToCharArray());
				
				//Start recording thread
				recordingThread = new Thread(new ThreadStart(RecordSamples));
				recordingThread.IsBackground = true;
				recordingThread.Start();
			}
			else
			{
				recording = false;
				recordingRunning = false;
			}
			return recording;
		}
		
		public void Pause()
		{
			recording = false;
		}
		
		public void Resume()
		{
			recording = true;
		}
		
		public void Stop()
		{
			readyToRecord = false;
			recording = false;
			//Stop recording thread
			recordingRunning = false;
			Thread.Sleep(100);
            if (recordingThread != null)
                recordingThread.Abort();
				
			//Stop MediaPipe stream
			mediaPipe.SendCmd("STO".ToCharArray());

			//Close file
			if (logFileStream!=null)
				logFileStream.Dispose();
		}
		
		private void RecordSamples()
		{
			while(recordingRunning)
			{
				if(recording)
				{
					//Write common time first
					logFileStream.Write(DateTime.Now.ToString("HH:mm:ss.fff") + ",");
					
					//Write robot data
					robot.Update(); // force update of state values
					logFileStream.Write((float)robot.State["t"][0] + ","+
										(float)robot.State["X"][0] + ","+
										(float)robot.State["X"][1] + ","+
										(float)robot.State["X"][2] + ","+
										(float)robot.State["dX"][0] + ","+
										(float)robot.State["dX"][1] + ","+
										(float)robot.State["dX"][2] + ","+
										(float)robot.State["F"][0] + ","+
										(float)robot.State["F"][1] + ","+
										(float)robot.State["F"][2] + ","+
										robot.State["Command"][0].ToString("0") + ","+
										robot.State["MvtProgress"][0].ToString("0") + ","+
										robot.State["Contribution"][0].ToString("0"));

					//Write trakSTAR sensor
					trakSTAR.Record_t[] records = new trakSTAR.Record_t[nbTrakstarSensors];
					trakstar.GetSensorsRecords(records);//update sensor records
					foreach(trakSTAR.Record_t r in records)
					{
						logFileStream.Write("," + (float)r.x + "," + (float)r.y + "," + (float)r.z + "," + (float)r.a + "," + (float)r.e + "," + (float)r.r);
					}
					
					//Write mediaPipe output
					if(mediaPipe.IsReceivedValues())
					{
						double[] vals = mediaPipe.GetReceivedValues();
						foreach(double val in vals)
						{
							logFileStream.Write("," + (float)val);
						}
					}
					else
					{
						//Write NaN instead	
						//TODO
					}
					
					logFileStream.Write("\n");
				}
				//Sleep for 5ms
				Thread.Sleep(5);
			}
		}
	}
}