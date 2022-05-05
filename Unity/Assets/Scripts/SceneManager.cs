using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.EventSystems;
using CORC;


public class SceneManager : MonoBehaviour
{
    public CORCM3_FOURIER Robot;
    
    public Button ToJerkBt, ToPathBt;
    public GameObject Cursor, Arrow;
    public InputField x_val, y_val, z_val, T_val, x_val_path, y_val_path, z_val_path;
    
    private Text Status;
    private InputField InputReturnCmd;

    double last_t = 0;

    // Start is called before the first frame update
    void Start()
    {
        //Admin panel elements
        InputField IPInput = GameObject.Find("AdminPanel/IPInput").GetComponent<InputField>();
        //IPInput.text = "192.168.7.2";
        IPInput.text = "127.0.0.1";
        
        Button ConnectBt = GameObject.Find("AdminPanel/ConnectBt").GetComponent<Button>();
        ConnectBt.onClick.AddListener(() => { Connect(ConnectBt, IPInput); });
        
        InputField InputCmd = GameObject.Find("AdminPanel/CmdInput").GetComponent<InputField>();
        InputCmd.text = "GOCA";
        
        InputReturnCmd = GameObject.Find("AdminPanel/ReturnCmdInput").GetComponent<InputField>();
        Button CmdBt = GameObject.Find("AdminPanel/CmdBt").GetComponent<Button>();
        CmdBt.onClick.AddListener(() => { SendCommand(CmdBt, InputCmd); });
        
        Status = GameObject.Find("AdminPanel/Status").GetComponent<Text>();
        
        
        //Session state panel
        Button QuitBt = GameObject.Find("SessionPanel/QuitBt").GetComponent<Button>();
        QuitBt.onClick.AddListener(() => { Quit(); });
        
        
        //Control panel
        Button LockBt = GameObject.Find("ControlPanel/LockBt").GetComponent<Button>();
        LockBt.onClick.AddListener(() => { Lock(LockBt); });
        
        Slider MassSl = GameObject.Find("ControlPanel/MassSl").GetComponent<Slider>();
        MassSl.onValueChanged.AddListener(delegate { UpdateMassSlider(MassSl.value, MassSl); });
        EventTrigger e = MassSl.GetComponentInChildren<EventTrigger>();//Messing up slider events: Unity sucks at managing events easily
        EventTrigger.Entry entry = new EventTrigger.Entry();
        entry.eventID = EventTriggerType.PointerUp;
        entry.callback.AddListener(delegate { ChangeMass(MassSl.value, MassSl); });
        e.triggers.Add(entry);
        
        Button GoGravBt = GameObject.Find("ControlBtLayout/GoGravBt").GetComponent<Button>();
        GoGravBt.onClick.AddListener(() => { GoGrav(MassSl.value); });
        
        GameObject PtsLayout = GameObject.Find("ControlPanel/PtsLayout");
        Button GoJerkBt = GameObject.Find("JerkLayout/GoJerkBt").GetComponent<Button>();
        GoJerkBt.onClick.AddListener(() => { GoJerk(PtsLayout); });
        
        Slider AssistanceSl = GameObject.Find("PathLayout/AssistanceSl").GetComponent<Slider>();
        AssistanceSl.onValueChanged.AddListener(delegate { UpdatePathAssistanceSlider(AssistanceSl.value, AssistanceSl); });
        e = AssistanceSl.GetComponentInChildren<EventTrigger>();//Messing up slider events: Unity sucks at managing events easily
        entry = new EventTrigger.Entry();
        entry.eventID = EventTriggerType.PointerUp;
        entry.callback.AddListener(delegate { ChangePathAssistance(AssistanceSl.value, AssistanceSl); });
        e.triggers.Add(entry);
        
        Button GoPathBt = GameObject.Find("PathLayout/GoPathBt").GetComponent<Button>();
        GoPathBt.onClick.AddListener(() => { GoPath(PtsLayout, AssistanceSl.value); });
        //Add pt
        string bt_path="PtsLayout/0/";
        Button AddPtBt = GameObject.Find(bt_path+"AddPtBt").GetComponent<Button>();
        AddPtBt.onClick.AddListener(() => { AddPt(AddPtBt); });
        //del pt
        Button DelPtBt = GameObject.Find(bt_path+"DelPtBt").GetComponent<Button>();
        DelPtBt.onClick.AddListener(() => { DelPt(DelPtBt); });
        
        //TODO: excepetion/error and proper naming and logging values
        //Robot.SetLoggingFile("mylog.csv");
        
        //Disable command panel
        enablePanel("ControlPanel", false);
    }

    // Update is called once per frame
    void Update()
    {
        Status.text = "Status:";
        if (Robot.IsInitialised())
        {
            //Update status text box
            Status.text += " Connected\n";
            Status.text += "\tt: " + Robot.State["t"][0].ToString("####.00") + " (" + (Robot.State["t"][0]-last_t).ToString("0.000") + ")\n";
            last_t= Robot.State["t"][0];
            Status.text += "\tX:";
            foreach(double val in Robot.State["X"])
                Status.text += val.ToString("0.000") + " \t";
            Status.text += "\n";
            Status.text += "\tdX:";
            foreach (double val in Robot.State["dX"])
                Status.text += val.ToString("0.00") + " \t";
            Status.text += "\n";
            Status.text += "\tF:";
            foreach (double val in Robot.State["F"])
                Status.text += val.ToString("00.0") + " \t";
            Status.text += "\n";
            Status.text += "Command: ";
            Status.text += Robot.State["Command"][0].ToString("0");
            Status.text += "\n";
            Status.text += "Mvt progress: ";
            Status.text += Robot.State["MvtProgress"][0].ToString("0.0");
            Status.text += "\n";
            Status.text += "Contribution: ";
            Status.text += Robot.State["Contribution"][0].ToString("0.0");
            Status.text += "\n";
            
            //Update UI
            updateContribution(Robot.State["Contribution"][0]);
            updatePtsProgress(Robot.State["MvtProgress"][0]);

            //Map cursor position and force interaction vector to current robot values
            float scale = 1000;
            Vector3 Origin = new Vector3(0, 80, -500);
            Cursor.transform.position = new Vector3((float)Robot.State["X"][1], (float)Robot.State["X"][2], -(float)Robot.State["X"][0])*scale+Origin;
            Vector3 force = new Vector3((float)Robot.State["F"][1], (float)Robot.State["F"][2], -(float)Robot.State["F"][0]);
            float force_scale = 10;
            Arrow.transform.localPosition = new Vector3(0, 0, force.magnitude / force_scale);
            Arrow.transform.localScale = new Vector3(0.2f, force.magnitude / force_scale, 0.2f);
            Cursor.transform.LookAt(Cursor.transform.position - force);
        }
        else
        {
            Status.text += " Not Connected\n";
            enablePanel("ControlPanel", false);
        }
    }

    void enablePanel(string panel_path, bool enable)
    {
        GameObject.Find(panel_path).GetComponent<CanvasGroup>().alpha = enable ? 1: 0.2f;
        GameObject.Find(panel_path).GetComponent<CanvasGroup>().interactable = enable;
    }
    
    void AddPt(Button bt)
    {
        //Fill in pt value
        string vals_path = "PtsLayout/"+bt.transform.parent.name+"/VertLayout/PtLayout/";
        GameObject.Find(vals_path+"x_val").GetComponent<InputField>().text = Robot.State["X"][0].ToString(".000");
        GameObject.Find(vals_path+"y_val").GetComponent<InputField>().text = Robot.State["X"][1].ToString(".000");
        GameObject.Find(vals_path+"z_val").GetComponent<InputField>().text = Robot.State["X"][2].ToString(".000");
        GameObject.Find(vals_path+"T_val").GetComponent<InputField>().text = "1.0"; //default 1 second
        
        //Add a new Pt if required
        int bt_idx = int.Parse(bt.transform.parent.name);
        GameObject pts_list = GameObject.Find("PtsLayout");
        
        if(pts_list.transform.childCount<4 && bt_idx+1 == pts_list.transform.childCount) {
            GameObject pt = GameObject.Find("PtsLayout/"+bt.transform.parent.name);
            GameObject new_pt = Instantiate(pt, pt.transform.parent);
            new_pt.name = (bt_idx+1).ToString("0");
            string bt_path="PtsLayout/"+new_pt.name+"/";
            Button AddPtBt = GameObject.Find(bt_path+"AddPtBt").GetComponent<Button>();
            AddPtBt.onClick.AddListener(() => { AddPt(AddPtBt); });
            Button DelPtBt = GameObject.Find(bt_path+"DelPtBt").GetComponent<Button>();
            DelPtBt.onClick.AddListener(() => { DelPt(DelPtBt); });
            //Clear values
            vals_path = "PtsLayout/"+new_pt.name+"/VertLayout/PtLayout/";
            GameObject.Find(vals_path+"x_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"y_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"z_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"T_val").GetComponent<InputField>().text="";
            //Disable delete on previous pt
            GameObject.Find("PtsLayout/"+bt.transform.parent.name+"/DelPtBt").GetComponent<Button>().interactable = false;
        }
    }
    
    void DelPt(Button bt)
    {
        //Remove pt (except if 1st one: clear only)
        int bt_idx = int.Parse(bt.transform.parent.name);
        GameObject pts_list = GameObject.Find("PtsLayout");
        Debug.Log(pts_list.transform.childCount);
        
        if(bt_idx>0) {
            GameObject pt = GameObject.Find("PtsLayout/"+(bt_idx).ToString("0"));
            Destroy(pt);
            //Re-activate del button prev button
            GameObject.Find("PtsLayout/"+(bt_idx-1).ToString("0")+"/DelPtBt").GetComponent<Button>().interactable = true;
        }
        else
        {
            //Clear values
            string vals_path = "PtsLayout/"+"0"+"/VertLayout/PtLayout/";
            GameObject.Find(vals_path+"x_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"y_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"z_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"T_val").GetComponent<InputField>().text="";
            //Re-activate del button prev button
            GameObject.Find("PtsLayout/"+"0"+"/DelPtBt").GetComponent<Button>().interactable = true;
        }
    }
    
    void updatePtsProgress(double progress)
    {
        //progress is in the form of [Mvt].percent. e.g. 1.5: half of movement from pt 1 to 2
        GameObject pts = GameObject.Find("PtsLayout");
        int idx = 0;
        foreach (Transform pt in pts.transform)
        {
            if(progress>idx+1) //Mvt completed
            {
                pt.GetComponentInChildren<Slider>().value=1.0f;
            }
            else if (progress>idx) //partial mvt
            {
                pt.GetComponentInChildren<Slider>().value = (float)(progress - (int)progress);
            }
            else
            {
                pt.GetComponentInChildren<Slider>().value = 0.0f;
            }
            idx++;
        }
    }
    
    void updateContribution(double contrib)
    {
        GameObject.Find("JerkLayout/ContributionSl").GetComponent<Slider>().value=(float)contrib;
    }
    
    double [] getPts()
    {
        List<double> p = new List<double>(); //Command parameters list
        
        GameObject pts_list = GameObject.Find("PtsLayout");
        int nb_pts = 0;
        foreach (Transform pt in pts_list.transform)
        {
          //Check if valid point (w/ values)
          if(GameObject.Find("PtsLayout/"+pt.name+"/VertLayout/PtLayout/"+"x_val").GetComponent<InputField>().text != "")
          {
              nb_pts++;
          }
        }
        //Nb pts parameter
        p.Add(nb_pts);
        //Pts values
        for (int i=0; i<nb_pts; i++ )
        {
            string pt_path="PtsLayout/"+i.ToString("0")+"/VertLayout/PtLayout/";
            //x
            InputField inp = GameObject.Find(pt_path+"x_val").GetComponent<InputField>();
            p.Add(double.Parse(inp.text));
            //y
            inp = GameObject.Find(pt_path+"y_val").GetComponent<InputField>();
            p.Add(double.Parse(inp.text));
            //z
            inp = GameObject.Find(pt_path+"z_val").GetComponent<InputField>();
            p.Add(double.Parse(inp.text));
            //T
            inp = GameObject.Find(pt_path+"T_val").GetComponent<InputField>();
            p.Add(double.Parse(inp.text));
        }
        
        return p.ToArray();
    }

    private IEnumerator UpdateRetCmd()
    {
        yield return new WaitForSeconds(0.1f);
        InputReturnCmd.text = Robot.GetCmd();
        if(InputReturnCmd.text.Contains("OK"))
        {
             GameObject.Find("CmdSuccessSnd").GetComponent<AudioSource>().Play();
        }
        else
        {
            GameObject.Find("CmdFailSnd").GetComponent<AudioSource>().Play();
        }
    }

    void GTNSCommand()
    {
        Robot.SendCmd("GTNS");
        StartCoroutine(UpdateRetCmd());
    }
    
    void SendCommand(Button CmdBt, InputField InputCmd)
    {
        Robot.SendCmd(InputCmd.text);
        StartCoroutine(UpdateRetCmd());
    }
    
    void GoJerk(GameObject Pts)
    {
        Robot.SendCmd("GOJE", getPts());
        StartCoroutine(UpdateRetCmd());
    }

    void GoPath(GameObject Pts, double assist)
    {
        Robot.SendCmd("GOPA", getPts());
        StartCoroutine(UpdateRetCmd());
    }

    void GoGrav(double val)
    {
        double [] p = {val};
        Robot.SendCmd("GOGR", p);
        StartCoroutine(UpdateRetCmd());
    }

    void Lock(Button bt)
    {
        if(bt.GetComponentInChildren<Text>().text == "Lock") {
            Robot.SendCmd("GOLO");
            bt.GetComponentInChildren<Text>().text = "Unlock";
            StartCoroutine(UpdateRetCmd());
        }
        else {
            Robot.SendCmd("GOUN");
            bt.GetComponentInChildren<Text>().text = "Lock";
            StartCoroutine(UpdateRetCmd());
        }
    }

    //Update mass value on slider WITHOUT sending the command
    void UpdateMassSlider(float v, Slider sl)
    {
        sl.GetComponentInChildren<TMPro.TextMeshProUGUI>().text=v.ToString("0.0")+"kg";
    }

    //Actually send changed mass value (on slider release only)
    public void ChangeMass(/*PointerEventData data, */float v, Slider sl)
    {
        sl.GetComponentInChildren<TMPro.TextMeshProUGUI>().text=v.ToString("0.0")+"kg";
        double [] p = {v};
        Robot.SendCmd("UDMA", p);
        StartCoroutine(UpdateRetCmd());
    }    
    
    //Update assistance value on slider WITHOUT sending the command
    public void UpdatePathAssistanceSlider(float v, Slider sl)
    {
        sl.GetComponentsInChildren<TMPro.TextMeshProUGUI>()[1].text=v.ToString("0.0");
    }
    
    //Actually send changed assistance value (on slider release only)
    public void ChangePathAssistance(float v, Slider sl)
    {
        sl.GetComponentsInChildren<TMPro.TextMeshProUGUI>()[1].text=v.ToString("0.0");
        double [] p = {v};
        Robot.SendCmd("UDPA", p);
        StartCoroutine(UpdateRetCmd());
    }

    void Connect(Button bt, InputField ip)
    {
        if (!Robot.IsInitialised())
        {
            Robot.Init(ip.text);
            if(Robot.IsInitialised()) {
                GameObject.Find("ConnectSuccessSnd").GetComponent<AudioSource>().Play();
                bt.GetComponentInChildren<Text>().text = "Disconnect";
                enablePanel("ControlPanel", true);
            }
        }
        else
        {
            Robot.Disconnect();
            bt.GetComponentInChildren<Text>().text = "Connect";
            enablePanel("ControlPanel", false);
        }
    }
    
    void Quit()
    {
        Application.Quit();
    }
    
    void OnApplicationQuit() 
    {
        if (Robot.IsInitialised())
        {
            GoGrav(.0);
            Robot.Disconnect();
        }
    }
}
