using System;
using System.Collections;
using System.Collections.Generic;
using System.Xml;
using System.Xml.Serialization;
using System.Text;
using System.IO;


namespace SessionsData
{
    public struct ActivityData
    {
        //Initialise to 0
        public ActivityData(string t, double a, double g)
        {
            type = t;
            distance = 0;
            nb_mvts = 0;
            assistance = a;
            gravity = g;
            time_s = 0;
            start_time = DateTime.Now;
        }
        
        public void CalculateFinalTime()
        {
            TimeSpan interval = DateTime.Now - start_time;
            time_s = (float)interval.TotalSeconds;
        }
        
        public double GetTime()
        {
            TimeSpan interval = DateTime.Now - start_time;
            return interval.TotalSeconds;
        }

        public string type; //Command use
        public float time_s;
        DateTime start_time;
        public double distance;
        public int nb_mvts;
        public double assistance;
        public double gravity;
    }

    public struct SessionData
    {
        //Initialise to 0
        public SessionData(int p)
        {
            patient_id = p;
            activities = new List<ActivityData>();
            start_time = DateTime.Now;
        }
        
        public void AddActivity(ActivityData a)
        {
            a.CalculateFinalTime();
            activities.Add(a);
        }
        
        public void WriteToXML()
        {
            string folder = "Patient"+patient_id.ToString("00");
            Directory.CreateDirectory(folder);
            string filename = folder+"/Patient"+patient_id.ToString("00")+"_"+start_time.ToString("dd-MM-yy_HH-mm-ss");
            XmlSerializer writer = new XmlSerializer(activities.GetType());
            StreamWriter file = new StreamWriter(filename+".xml");
            writer.Serialize(file, activities);
            file.Close();
        }
        
        int patient_id;
        DateTime start_time;
        public List<ActivityData> activities;
    }
}