//Please, if you use this, share the improvements

using AgOpenGPS.Classes;
using AgOpenGPS.Properties;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Reflection;
using System.Resources;
using System.Windows.Forms;

namespace AgOpenGPS
{
    //the main form object
    public partial class FormGPS : Form
    {
        //To bring forward AgIO if running
        [System.Runtime.InteropServices.DllImport("User32.dll")]
        private static extern bool SetForegroundWindow(IntPtr handle);

        [System.Runtime.InteropServices.DllImport("User32.dll")]
        private static extern bool ShowWindow(IntPtr hWind, int nCmdShow);

        #region // Class Props and instances

        //maximum sections available
        public const int MAXSECTIONS = 64;

        //How many boundaries allowed
        public const int MAXBOUNDARIES = 6;

        //How many headlands allowed
        public const int MAXHEADS = 6;

        internal void button1_Click()
        {
            throw new NotImplementedException();
        }

        //The base directory where AgOpenGPS will be stored and fields and vehicles branch from
        public string baseDirectory;

        //current directory of vehicle
        public string vehiclesDirectory, vehicleFileName = "";

        //current directory of tools
        public string toolsDirectory, toolFileName = "";

        //current directory of Environments
        public string envDirectory, envFileName = "";

        //current fields and field directory
        public string fieldsDirectory, currentFieldDirectory, displayFieldName;

        private bool leftMouseDownOnOpenGL; //mousedown event in opengl window
        public int flagNumberPicked = 0;

        //bool for whether or not a job is active
        public bool isJobStarted = false, isBtnAutoSteerOn, isLidarBtnOn = true;

        //if we are saving a file
        public bool isSavingFile = false, isLogNMEA = false;

        //texture holders
        public uint[] texture;

        //the currentversion of software
        public string currentVersionStr, inoVersionStr;

        public int inoVersionInt;

        //create instance of a stopwatch for timing of frames and NMEA hz determination
        private readonly Stopwatch swFrame = new Stopwatch();

        public double secondsSinceStart;
        public double gridToolSpacing;

        //private readonly Stopwatch swDraw = new Stopwatch();
        //swDraw.Reset();
        //swDraw.Start();
        //swDraw.Stop();
        //label3.Text = ((double) swDraw.ElapsedTicks / (double) System.Diagnostics.Stopwatch.Frequency * 1000).ToString();

        //Time to do fix position update and draw routine
        public double frameTime = 0;

        //create instance of a stopwatch for timing of frames and NMEA hz determination
        //private readonly Stopwatch swHz = new Stopwatch();

        //Time to do fix position update and draw routine
        public double gpsHz = 10;

        //whether or not to use Stanley control
        public bool isStanleyUsed = true, isSimPosfixed = true, isFlagMenuOpen = false;

        public int pbarSteer, pbarMachine, pbarUDP;

        public double nudNumber = 0;

        public double m2InchOrCm, inchOrCm2m, m2FtOrM, ftOrMtoM, cm2CmOrIn, inOrCm2Cm;
        public string unitsFtM, unitsInCm, unitsInCmNS;

        public char[] hotkeys;

        //used by filePicker Form to return picked file and directory
        public string filePickerFileAndDirectory;

        //the position of the GPS Data window within the FormGPS window
        public int GPSDataWindowLeft = 80, GPSDataWindowTopOffset = 220;

        //the autoManual drive button. Assume in Auto
        public bool isInAutoDrive = true;

        //drive after ContourPattern lines
        public bool isContourPattern = false;
        public bool isSlopeline = false, isThirdAntenne = false;
        public bool isNewLineCurve = false;
        public double MaxDistanceSunPattern, DistanceA1B1, DistanceA2B2;
        public int howManyPathPattern, isData_Refresh = 3;
        public int HeadingMPU6050, RollMPU6050, HeadingFilterMPU6050, RollFilterMPU6050;



        //isGPSData form up
        public bool isGPSSentencesOn = false, isKeepOffsetsOn = false, lblGoneDisdanceClick = false;

        public double uturnFromBoundaryA, uturnFromBoundaryB;
        public double nudgeDistanceA, nudgeDistanceB;

        /// <summary>
        /// create the scene camera
        /// </summary>
        public CCamera camera = new CCamera();

        /// <summary>
        /// create world grid
        /// </summary>
        public CWorldGrid worldGrid;

        /// <summary>
        /// The NMEA class that decodes it
        /// </summary>
        public CNMEA pn;

        /// <summary>
        /// an array of sections
        /// </summary>
        public CSection[] section;

        /// <summary>
        /// an array of patches to draw
        /// </summary>
        //public CPatches[] triStrip;
        public List<CPatches> triStrip;
        public List<CPatches> triStripSC;

        /// <summary>
        /// AB Line object
        /// </summary>
        public CABLine ABLine;

        public CABLine distanzABLine1;
        public CABLine distanzABLine2;

        /// <summary>
        /// AB Contour Line object
        /// </summary>
        public CContourLines Cont;

        /// <summary>
        /// TramLine class for boundary and settings
        /// </summary>
        public CTram tram;

        /// <summary>
        /// TramLine class for boundary and settings
        /// </summary>
        public CTramline traml;

        /// <summary>
        /// Contour Mode Instance
        /// </summary>
        public CContour ct;

        /// <summary>
        /// Contour Mode Instance
        /// </summary>
        public CTrack trk;

        /// <summary>
        /// Contour Mode Instance
        /// </summary>
        public CToolTrack tooltrk;

        /// <summary>
        /// ABCurve instance
        /// </summary>
        public CABCurve curve;

        /// <summary>
        /// Auto Headland YouTurn
        /// </summary>
        public CYouTurn yt;

        /// <summary>
        /// Building a YouTrun instance
        /// </summary>
        public CYouTurnLine Ytl;

        /// <summary>
        /// Our vehicle only
        /// </summary>
        public CVehicle vehicle;

        /// <summary>
        /// Just the tool attachment that includes the sections
        /// </summary>
        public CTool tool;

        /// <summary>
        /// All the structs for recv and send of information out ports
        /// </summary>
        public CModuleComm mc;

        /// <summary>
        /// The boundary object
        /// </summary>
        public CBoundary bnd;

        /// <summary>
        /// The boundary object
        /// </summary>
        public CBoundaryLine bndl;

        /// <summary>
        /// Building a headland instance
        /// </summary>
        public CHeadLine hdl;

        /// <summary>
        /// The internal simulator
        /// </summary>
        public CSim sim;

        /// <summary>
        /// Resource manager for gloabal strings
        /// </summary>
        public ResourceManager _rm;

        /// <summary>
        /// Heading, Roll, Pitch, GPS, Properties
        /// </summary>
        public CAHRS ahrs;

        /// <summary>
        /// Recorded Path
        /// </summary>
        public CRecordedPath recPath;

        /// <summary>
        /// Recorded Tool Track
        /// </summary>
       // public CToolTrack recTool;

        /// <summary>
        /// Most of the displayed field data for GUI
        /// </summary>
        public CFieldData fd;

        ///// <summary>
        ///// Sound
        ///// </summary>
        public CSound sounds;

        /// <summary>
        /// The font class
        /// </summary>
        public CFont font;

        public ShapeFile shape;
        /// <summary>
        /// The new brightness code
        /// </summary>

        /// <summary>
        /// The new steer algorithms
        /// </summary>
        public CGuidance gyd;

        /// <summary>
        /// The new brightness code
        /// </summary>
        public CWindowsSettingsBrightnessController displayBrightness;

        #endregion // Class Props and instances

        public FormGPS()
        {
            //winform initialization
            InitializeComponent();

            CheckSettingsNotNull();

            //time keeper
            secondsSinceStart = (DateTime.Now - Process.GetCurrentProcess().StartTime).TotalSeconds;

            //create the world grid
            worldGrid = new CWorldGrid(this);

            //our vehicle made with gl object and pointer of mainform
            vehicle = new CVehicle(this);

            tool = new CTool(this);

            //create a new section and set left and right positions
            //created whether used or not, saves restarting program

            section = new CSection[MAXSECTIONS];
            for (int j = 0; j < MAXSECTIONS; j++) section[j] = new CSection();

            triStrip = new List<CPatches>
            {
                new CPatches(this)
            };

            triStripSC = new List<CPatches>
            {
                new CPatches(this)
            };

            //our NMEA parser
            pn = new CNMEA(this);

            //create the ABLine instance
            ABLine = new CABLine(this);

            distanzABLine1 = new CABLine(this);
            distanzABLine2 = new CABLine(this);

            //new instance of contour mode
            ct = new CContour(this);

            //new instance of contour mode
            Cont = new CContourLines(this);

            //new instance of contour mode
            curve = new CABCurve(this);
 
            //new track instance
            trk = new CTrack(this);

            //new tooltrack instance
            tooltrk = new CToolTrack(this);

            //new instance of contour mode
            hdl = new CHeadLine(this);

            ////new instance of auto headland turn
            yt = new CYouTurn(this);

            Ytl = new CYouTurnLine(this);

            //module communication
            mc = new CModuleComm(this);

            //boundary object
            bnd = new CBoundary(this);

            //boundary object
            bndl = new CBoundaryLine(this);

            //nmea simulator built in.
            sim = new CSim(this);

            ////all the attitude, heading, roll, pitch reference system
            ahrs = new CAHRS();

            //A recorded path
            recPath = new CRecordedPath(this);

            //fieldData all in one place
            fd = new CFieldData(this);

            //start the stopwatch
            //swFrame.Start();

            //instance of tram
            tram = new CTram(this);

            //instance of tramline
            traml = new CTramline(this);

            //resource for gloabal language strings
            _rm = new ResourceManager("AgOpenGPS.gStr", Assembly.GetExecutingAssembly());

            //access to font class
            font = new CFont(this);

            //the new steer algorithms
            gyd = new CGuidance(this);

            //sounds class
            sounds = new CSound();

            //brightness object class
            displayBrightness = new CWindowsSettingsBrightnessController(Properties.Settings.Default.setDisplay_isBrightnessOn);

            //shape file object
            shape = new ShapeFile(this);
        }

        private void FormGPS_Load(object sender, EventArgs e)
        {
            this.MouseWheel += ZoomByMouseWheel;

            //start udp server is required
            StartLoopbackServer();

            //boundaryToolStripBtn.Enabled = false;
            FieldMenuButtonEnableDisable(false);

            panelRight.Enabled = false;

            oglMain.Left = 75;
            oglMain.Width = this.Width - statusStripLeft.Width - 84;

            panelSim.Left = Width / 2 - 330;
            panelSim.Width = 700;
            panelSim.Top = Height - 60;

            //set the language to last used
            SetLanguage(Settings.Default.setF_culture, false);

            currentVersionStr = Application.ProductVersion.ToString(CultureInfo.InvariantCulture);

            string[] fullVers = currentVersionStr.Split('.');
            int inoV = int.Parse(fullVers[0], CultureInfo.InvariantCulture);
            inoV += int.Parse(fullVers[1], CultureInfo.InvariantCulture);
            inoV += int.Parse(fullVers[2], CultureInfo.InvariantCulture);
            inoVersionInt = inoV;
            inoVersionStr = inoV.ToString();

            if (Settings.Default.setF_workingDirectory == "Default")
                baseDirectory = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + "\\AgOpenGPS\\";
            else baseDirectory = Settings.Default.setF_workingDirectory + "\\AgOpenGPS\\";

            //get the fields directory, if not exist, create
            fieldsDirectory = baseDirectory + "Fields\\";
            string dir = Path.GetDirectoryName(fieldsDirectory);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir)) { Directory.CreateDirectory(dir); }

            //get the fields directory, if not exist, create
            vehiclesDirectory = baseDirectory + "Vehicles\\";
            dir = Path.GetDirectoryName(vehiclesDirectory);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir)) { Directory.CreateDirectory(dir); }

            //get the abLines directory, if not exist, create
            ablinesDirectory = baseDirectory + "ABLines\\";
            dir = Path.GetDirectoryName(fieldsDirectory);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir)) { Directory.CreateDirectory(dir); }

            //make sure current field directory exists, null if not
            currentFieldDirectory = Settings.Default.setF_CurrentDir;

            btnBuildTracks_small.Image = Properties.Resources.Splitlines;
            isContourPattern = false;
            tool.isToolAntenna3 = false;

            string curDir;
            if (currentFieldDirectory != "")
            {
                curDir = fieldsDirectory + currentFieldDirectory + "//";
                dir = Path.GetDirectoryName(curDir);
                if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                {
                    currentFieldDirectory = "";
                    Settings.Default.setF_CurrentDir = "";
                    Settings.Default.Save();
                }
            }

            if (isBrightnessOn)
            {
                if (displayBrightness.isWmiMonitor)
                {
                    Settings.Default.setDisplay_brightnessSystem = displayBrightness.GetBrightness();
                    Settings.Default.Save();
                }
                else
                {
                    btnBrightnessDn.Enabled = false;
                    btnBrightnessUp.Enabled = false;
                }

                //display brightness
                if (displayBrightness.isWmiMonitor)
                {
                    if (Settings.Default.setDisplay_brightness < Settings.Default.setDisplay_brightnessSystem)
                    {
                        Settings.Default.setDisplay_brightness = Settings.Default.setDisplay_brightnessSystem;
                        Settings.Default.Save();
                    }

                    displayBrightness.SetBrightness(Settings.Default.setDisplay_brightness);
                }
                else
                {
                    btnBrightnessDn.Enabled = false;
                    btnBrightnessUp.Enabled = false;
                }
            }

            // load all the gui elements in gui.designer.cs
            LoadSettings();

            //for field data and overlap
            oglZoom.Width = 400;
            oglZoom.Height = 400;
            oglZoom.Left = 100;
            oglZoom.Top = 100;

            if (Properties.Settings.Default.setDisplay_isAutoStartAgIO)
            {
                //Start AgIO process
                Process[] processName = Process.GetProcessesByName("AgIO");
                if (processName.Length == 0)
                {
                    //Start application here
                    DirectoryInfo di = new DirectoryInfo(Application.StartupPath);
                    string strPath = di.ToString();
                    strPath += "\\AgIO.exe";
                    try
                    {
                        ProcessStartInfo processInfo = new ProcessStartInfo
                        {
                            FileName = strPath,
                            WorkingDirectory = Path.GetDirectoryName(strPath)
                        };
                        Process proc = Process.Start(processInfo);
                    }
                    catch
                    {
                        TimedMessageBox(2000, "No File Found", "Can't Find AgIO");
                    }
                }
            }

            //nmea limiter
            udpWatch.Start();

            ControlExtension.Draggable(panelDrag, true);

            setWorkingDirectoryToolStripMenuItem.Text = gStr.gsDirectories;
            enterSimCoordsToolStripMenuItem.Text = gStr.gsEnterSimCoords;
            aboutToolStripMenuItem.Text = gStr.gsAbout;
            menustripLanguage.Text = gStr.gsLanguage;

            simulatorOnToolStripMenuItem.Text = gStr.gsSimulatorOn;
            resetALLToolStripMenuItem.Text = gStr.gsResetAll;
            colorsToolStripMenuItem1.Text = gStr.gsColors;
            resetEverythingToolStripMenuItem.Text = gStr.gsResetAllForSure;
            steerChartStripMenu.Text = gStr.gsCharts;

            //Tools Menu
            SmoothABtoolStripMenu.Text = gStr.gsSmoothABCurve;
            boundariesToolStripMenuItem.Text = gStr.gsBoundary;
            headlandToolStripMenuItem.Text = gStr.gsHeadland;
            headlandBuildToolStripMenuItem.Text = gStr.gsHeadland + " (2)";
            deleteContourPathsToolStripMenuItem.Text = gStr.gsDeleteContourPaths;
            deleteAppliedToolStripMenuItem.Text = gStr.gsDeleteAppliedArea;
            tramLinesMenuField.Text = gStr.gsTramLines;
            recordedPathStripMenu.Text = gStr.gsRecordedPathMenu;
            flagByLatLonToolStripMenuItem.Text = gStr.gsFlagByLatLon;
            boundaryToolToolStripMenu.Text = gStr.gsBoundary + " Tool";

            webcamToolStrip.Text = gStr.gsWebCam;
            offsetFixToolStrip.Text = gStr.gsOffsetFix;
            wizardsMenu.Text = gStr.gsWizards;
            steerWizardMenuItem.Text = gStr.gsSteerWizard;
            steerChartToolStripMenuItem.Text = gStr.gsSteerChart;
            headingChartToolStripMenuItem.Text = gStr.gsHeadingChart;
            xTEChartToolStripMenuItem.Text = gStr.gsXTEChart;

            btnChangeMappingColor.Text = Application.ProductVersion.ToString(CultureInfo.InvariantCulture);
            //btnChangeMappingColor.Text = btnChangeMappingColor.Text.Substring(2);

            hotkeys = new char[19];

            hotkeys = Properties.Settings.Default.setKey_hotkeys.ToCharArray();

            if (!isTermsAccepted)
            {
                if (!Properties.Settings.Default.setDisplay_isTermsAccepted)
                {
                    using (var form = new Form_First(this))
                    {
                        if (form.ShowDialog(this) != DialogResult.OK)
                        {
                            Close();
                        }
                    }
                }
            }
        }

        private void FormGPS_FormClosing(object sender, FormClosingEventArgs e)
        {
            Form f = Application.OpenForms["FormGPSData"];

            if (f != null)
            {
                f.Focus();
                f.Close();
            }

            f = Application.OpenForms["FormFieldData"];

            if (f != null)
            {
                f.Focus();
                f.Close();
            }

            f = Application.OpenForms["FormPan"];

            if (f != null)
            {
                isPanFormVisible = false;
                f.Focus();
                f.Close();
            }

            if (this.OwnedForms.Any())
            {
                TimedMessageBox(2000, gStr.gsWindowsStillOpen, gStr.gsCloseAllWindowsFirst);
                e.Cancel = true;
                return;
            }

            if (isJobStarted)
            {
                if (autoBtnState == btnStates.Auto)
                    btnSectionMasterAuto.PerformClick();

                if (manualBtnState == btnStates.On)
                    btnSectionMasterManual.PerformClick();

                bool closing = true;
                int choice = SaveOrNot(closing);

                if (choice == 1)
                {
                    e.Cancel = true;
                    return;
                }

                //Save, return, cancel save
                if (isJobStarted)
                {
                    if (choice == 3)
                    {
                        e.Cancel = true;
                        return;
                    }
                    else if (choice == 0)
                    {
                        FileSaveEverythingBeforeClosingField();
                    }
                    if (choice == 2)  // ###############################
                    {
                        toolStripAreYouSure();
                        FileSaveEverythingBeforeClosingField();
                    }                 // ###############################
                }
            }

            SaveFormGPSWindowSettings();
            FileUpdateAllFieldsKML();

            if (loopBackSocket != null)
            {
                try
                {
                    loopBackSocket.Shutdown(SocketShutdown.Both);
                }
                catch { }
                finally { loopBackSocket.Close(); }
            }

            //save current vehicle
            SettingsIO.ExportAll(vehiclesDirectory + vehicleFileName + ".XML");

            if (displayBrightness.isWmiMonitor)
                displayBrightness.SetBrightness(Settings.Default.setDisplay_brightnessSystem);
        }

        public int SaveOrNot(bool closing)
        {
            CloseTopMosts();

            using (FormSaveOrNot form = new FormSaveOrNot(closing))
            {
                DialogResult result = form.ShowDialog(this);

                if (result == DialogResult.OK) return 0;      //Save and Exit
                if (result == DialogResult.Ignore) return 1;   //Ignore
                if (result == DialogResult.Yes) return 2;   //Ignore

                return 3;  // oops something is really busted
            }
        }

        private void FormGPS_ResizeEnd(object sender, EventArgs e)
        {
            PanelsAndOGLSize();
            if (isGPSPositionInitialized) SetZoom();

            Form f = Application.OpenForms["FormGPSData"];
            if (f != null)
            {
                f.Top = this.Top + this.Height / 2 - GPSDataWindowTopOffset;
                f.Left = this.Left + GPSDataWindowLeft;
            }

            f = Application.OpenForms["FormFieldData"];
            if (f != null)
            {
                f.Top = this.Top + this.Height / 2 - GPSDataWindowTopOffset;
                f.Left = this.Left + GPSDataWindowLeft;
            }

            f = Application.OpenForms["FormPan"];
            if (f != null)
            {
                f.Top = this.Top + 90;
                f.Left = this.Left + 120;
            }
        }

        // Return True if a certain percent of a rectangle is shown across the total screen area of all monitors, otherwise return False.
        public bool IsOnScreen(System.Drawing.Point RecLocation, System.Drawing.Size RecSize, double MinPercentOnScreen = 0.8)
        {
            double PixelsVisible = 0;
            System.Drawing.Rectangle Rec = new System.Drawing.Rectangle(RecLocation, RecSize);

            foreach (Screen Scrn in Screen.AllScreens)
            {
                System.Drawing.Rectangle r = System.Drawing.Rectangle.Intersect(Rec, Scrn.WorkingArea);
                // intersect rectangle with screen
                if (r.Width != 0 & r.Height != 0)
                {
                    PixelsVisible += (r.Width * r.Height);
                    // tally visible pixels
                }
            }
            return PixelsVisible >= (Rec.Width * Rec.Height) * MinPercentOnScreen;
        }

        private void FormGPS_Move(object sender, EventArgs e)
        {
            Form f = Application.OpenForms["FormGPSData"];
            if (f != null)
            {
                f.Top = this.Top + this.Height / 2 - GPSDataWindowTopOffset;
                f.Left = this.Left + GPSDataWindowLeft;
            }

            f = Application.OpenForms["FormFieldData"];
            if (f != null)
            {
                f.Top = this.Top + this.Height / 2 - GPSDataWindowTopOffset;
                f.Left = this.Left + GPSDataWindowLeft;
            }

            f = Application.OpenForms["FormPan"];
            if (f != null)
            {
                f.Top = this.Top + 75;
                f.Left = this.Left + this.Width - 380;
            }
        }

        public void CheckSettingsNotNull()
        {
            if (Settings.Default.setFeatures == null)
            {
                Settings.Default.setFeatures = new CFeatureSettings();
            }
        }

        public enum textures : uint
        {
            Floor, Font,
            Turn, TurnCancel, TurnManual,
            Compass, Speedo, SpeedoNeedle,
            Lift, SteerPointer,
            SteerDot, Tractor, QuestionMark,
            FrontWheels, FourWDFront, FourWDRear,
            Harvester,
            Lateral, bingGrid,
            NoGPS, ZoomIn48, ZoomOut48,
            Pan, MenuHideShow,
            ToolWheels, Tire, TramDot,
            RateMap1, RateMap2, RateMap3,
            YouTurnU, YouTurnH, CrossTrackBkgrnd
        }

        public void LoadGLTextures()
        {
            GL.Enable(EnableCap.Texture2D);

            Bitmap[] oglTextures = new Bitmap[]
            {
                Resources.z_Floor,Resources.z_Font,
                Resources.z_Turn,Resources.z_TurnCancel,Resources.z_TurnManual,
                Resources.z_Compass,Resources.z_Speedo,Resources.z_SpeedoNeedle,
                Resources.z_Lift,Resources.z_SteerPointer,
                Resources.z_SteerDot,GetTractorBrand(Settings.Default.setBrand_TBrand),Resources.z_QuestionMark,
                Resources.z_FrontWheels,Get4WDBrandFront(Settings.Default.setBrand_WDBrand),
                Get4WDBrandRear(Settings.Default.setBrand_WDBrand),
                GetHarvesterBrand(Settings.Default.setBrand_HBrand),
                Resources.z_LateralManual, Resources.z_bingMap,
                Resources.z_NoGPS, Resources.ZoomIn48, Resources.ZoomOut48,
                Resources.Pan, Resources.MenuHideShow,
                Resources.z_Tool, Resources.z_Tire, Resources.z_TramOnOff,
                Resources.z_RateMap1, Resources.z_RateMap2, Resources.z_RateMap3,
                Resources.YouTurnU, Resources.YouTurnH, Resources.z_crossTrackBkgnd
            };

            texture = new uint[oglTextures.Length];

            for (int h = 0; h < oglTextures.Length; h++)
            {
                using (Bitmap bitmap = oglTextures[h])
                {
                    GL.GenTextures(1, out texture[h]);
                    GL.BindTexture(TextureTarget.Texture2D, texture[h]);
                    BitmapData bitmapData = bitmap.LockBits(new Rectangle(0, 0, bitmap.Width, bitmap.Height), ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
                    GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba, bitmapData.Width, bitmapData.Height, 0, OpenTK.Graphics.OpenGL.PixelFormat.Bgra, PixelType.UnsignedByte, bitmapData.Scan0);
                    bitmap.UnlockBits(bitmapData);
                    GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, 9729);
                    GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, 9729);
                }
            }
        }

        public bool KeypadToNUD(NudlessNumericUpDown sender, Form owner)
        {
            var colour = sender.BackColor;
            sender.BackColor = Color.Red;
            sender.Value = Math.Round(sender.Value, sender.DecimalPlaces);
            using (FormNumeric form = new FormNumeric((double)sender.Minimum, (double)sender.Maximum, (double)sender.Value))
            {
                DialogResult result = form.ShowDialog(owner);
                if (result == DialogResult.OK)
                {
                    sender.Value = (decimal)form.ReturnValue;
                    sender.BackColor = colour;
                    return true;
                }
                else if (result == DialogResult.Cancel)
                {
                    sender.BackColor = colour;
                }
                return false;
            }
        }

        public void KeyboardToText(TextBox sender, Form owner)
        {
            var colour = sender.BackColor;
            sender.BackColor = Color.Red;
            using (FormKeyboard form = new FormKeyboard(sender.Text))
            {
                if (form.ShowDialog(owner) == DialogResult.OK)
                {
                    sender.Text = form.ReturnString;
                }
            }
            sender.BackColor = colour;
        }

        //request a new job
        public void JobNew()
        {
            //SendSteerSettingsOutAutoSteerPort();
            isJobStarted = true;
            startCounter = 0;

            btnFieldStats.Visible = true;

            btnSectionMasterManual.Enabled = true;
            manualBtnState = btnStates.Off;
            btnSectionMasterManual.Image = Properties.Resources.ManualOff;

            btnSectionMasterAuto.Enabled = true;
            autoBtnState = btnStates.Off;
            btnSectionMasterAuto.Image = Properties.Resources.SectionMasterOff;

            btnSection1Man.BackColor = Color.Red;
            btnSection2Man.BackColor = Color.Red;
            btnSection3Man.BackColor = Color.Red;
            btnSection4Man.BackColor = Color.Red;
            btnSection5Man.BackColor = Color.Red;
            btnSection6Man.BackColor = Color.Red;
            btnSection7Man.BackColor = Color.Red;
            btnSection8Man.BackColor = Color.Red;
            btnSection9Man.BackColor = Color.Red;
            btnSection10Man.BackColor = Color.Red;
            btnSection11Man.BackColor = Color.Red;
            btnSection12Man.BackColor = Color.Red;
            btnSection13Man.BackColor = Color.Red;
            btnSection14Man.BackColor = Color.Red;
            btnSection15Man.BackColor = Color.Red;
            btnSection16Man.BackColor = Color.Red;

            btnSection1Man.Enabled = true;
            btnSection2Man.Enabled = true;
            btnSection3Man.Enabled = true;
            btnSection4Man.Enabled = true;
            btnSection5Man.Enabled = true;
            btnSection6Man.Enabled = true;
            btnSection7Man.Enabled = true;
            btnSection8Man.Enabled = true;
            btnSection9Man.Enabled = true;
            btnSection10Man.Enabled = true;
            btnSection11Man.Enabled = true;
            btnSection12Man.Enabled = true;
            btnSection13Man.Enabled = true;
            btnSection14Man.Enabled = true;
            btnSection15Man.Enabled = true;
            btnSection16Man.Enabled = true;

            btnZone1.BackColor = Color.Red;
            btnZone2.BackColor = Color.Red;
            btnZone3.BackColor = Color.Red;
            btnZone4.BackColor = Color.Red;
            btnZone5.BackColor = Color.Red;
            btnZone6.BackColor = Color.Red;
            btnZone7.BackColor = Color.Red;
            btnZone8.BackColor = Color.Red;

            btnZone1.Enabled = true;
            btnZone2.Enabled = true;
            btnZone3.Enabled = true;
            btnZone4.Enabled = true;
            btnZone5.Enabled = true;
            btnZone6.Enabled = true;
            btnZone7.Enabled = true;
            btnZone8.Enabled = true;

            btnContour.Enabled = true;
            btnTrack.Enabled = true;
            btnABDraw.Enabled = true;
            btnCycleLines.Image = Properties.Resources.ABLineCycle;
            btnCycleLinesBk.Image = Properties.Resources.ABLineCycleBk;

            ABLine.abHeading = 0.00;
            btnAutoSteer.Enabled = true;

            DisableYouTurnButtons();
            btnFlag.Enabled = true;

            if (tool.isSectionsNotZones)
            {
                LineUpIndividualSectionBtns();
            }
            else
            {
                LineUpAllZoneButtons();
            }

            //update the menu
            this.menustripLanguage.Enabled = false;
            panelRight.Enabled = true;
            //boundaryToolStripBtn.Enabled = true;
            isPanelBottomHidden = false;

            FieldMenuButtonEnableDisable(true);
            PanelUpdateRightAndBottom();
            PanelsAndOGLSize();
            SetZoom();
            fileSaveCounter = 25;
            lblGuidanceLine.Visible = false;
            btnAutoTrack.Image = Resources.AutoTrackOff;
            trk.isAutoTrack = false;
            CalculateABLength();
        }

        //close the current job
        public void JobClose()
        {
            recPath.resumeState = 0;
            btnResumePath.Image = Properties.Resources.pathResumeStart;
            recPath.currentPositonIndex = 0;

            sbGrid.Clear();

            //reset field offsets
            if (!isKeepOffsetsOn)
            {
                pn.fixOffset.easting = 0;
                pn.fixOffset.northing = 0;
            }

            //turn off headland
            bnd.isHeadlandOn = false;

            btnFieldStats.Visible = false;

            recPath.recList.Clear();
            recPath.StopDrivingRecordedPath();
            panelDrag.Visible = false;

            //make sure hydraulic lift is off
            p_239.pgn[p_239.hydLift] = 0;
            vehicle.isHydLiftOn = false;
            btnHydLift.Image = Properties.Resources.HydraulicLiftOff;
            btnHydLift.Visible = false;

            lblGuidanceLine.Visible = false;

            //zoom gone
            oglZoom.SendToBack();

            //clean all the lines
            bnd.bndList.Clear();
            bnd.shpList.Clear();

            panelRight.Enabled = false;
            FieldMenuButtonEnableDisable(false);

            menustripLanguage.Enabled = true;
            isJobStarted = false;

            lblDistancetoBoundary.Visible = false;
            lblDistancefromBoundary.Visible = false;


            //fix ManualOffOnAuto buttons
            manualBtnState = btnStates.Off;
            btnSectionMasterManual.Image = Properties.Resources.ManualOff;

            //fix auto button
            autoBtnState = btnStates.Off;
            btnSectionMasterAuto.Image = Properties.Resources.SectionMasterOff;

            if (tool.isSectionsNotZones)
            {
                //Update the button colors and text
                AllSectionsAndButtonsToState(btnStates.Off);

                //enable disable manual buttons
                LineUpIndividualSectionBtns();
            }
            else
            {
                AllZonesAndButtonsToState(autoBtnState);
                LineUpAllZoneButtons();
            }

            btnZone1.BackColor = Color.Silver;
            btnZone2.BackColor = Color.Silver;
            btnZone3.BackColor = Color.Silver;
            btnZone4.BackColor = Color.Silver;
            btnZone5.BackColor = Color.Silver;
            btnZone6.BackColor = Color.Silver;
            btnZone7.BackColor = Color.Silver;
            btnZone8.BackColor = Color.Silver;

            btnZone1.Enabled = false;
            btnZone2.Enabled = false;
            btnZone3.Enabled = false;
            btnZone4.Enabled = false;
            btnZone5.Enabled = false;
            btnZone6.Enabled = false;
            btnZone7.Enabled = false;
            btnZone8.Enabled = false;

            btnSection1Man.Enabled = false;
            btnSection2Man.Enabled = false;
            btnSection3Man.Enabled = false;
            btnSection4Man.Enabled = false;
            btnSection5Man.Enabled = false;
            btnSection6Man.Enabled = false;
            btnSection7Man.Enabled = false;
            btnSection8Man.Enabled = false;
            btnSection9Man.Enabled = false;
            btnSection10Man.Enabled = false;
            btnSection11Man.Enabled = false;
            btnSection12Man.Enabled = false;
            btnSection13Man.Enabled = false;
            btnSection14Man.Enabled = false;
            btnSection15Man.Enabled = false;
            btnSection16Man.Enabled = false;

            btnSection1Man.BackColor = Color.Silver;
            btnSection2Man.BackColor = Color.Silver;
            btnSection3Man.BackColor = Color.Silver;
            btnSection4Man.BackColor = Color.Silver;
            btnSection5Man.BackColor = Color.Silver;
            btnSection6Man.BackColor = Color.Silver;
            btnSection7Man.BackColor = Color.Silver;
            btnSection8Man.BackColor = Color.Silver;
            btnSection9Man.BackColor = Color.Silver;
            btnSection10Man.BackColor = Color.Silver;
            btnSection11Man.BackColor = Color.Silver;
            btnSection12Man.BackColor = Color.Silver;
            btnSection13Man.BackColor = Color.Silver;
            btnSection14Man.BackColor = Color.Silver;
            btnSection15Man.BackColor = Color.Silver;
            btnSection16Man.BackColor = Color.Silver;

            //clear the section lists
            for (int j = 0; j < triStrip.Count; j++)
            {
                //clean out the lists
                triStrip[j].patchList?.Clear();
                triStrip[j].triangleList?.Clear();
            }

            triStrip?.Clear();
            triStrip.Add(new CPatches(this));

            //clear the flags
            flagPts.Clear();

            //ABLine
            tram.tramList?.Clear();
            //ABLine
            traml.tramList?.Clear();

            //curve line
            curve.ResetCurveLine();

            //tracks
            trk.gArr?.Clear();
            trk.idx = -1;

            //clean up tram
            tram.displayMode = 0;
            tram.generateMode = 0;
            tram.tramBndInnerArr?.Clear();
            tram.tramBndOuterArr?.Clear();
            traml.displayMode = 0;
            traml.generateMode = 0;
            traml.tramBndInnerArr?.Clear();
            traml.tramBndOuterArr?.Clear();

            //clear out contour and Lists
            btnContour.Enabled = false;
            //btnContourPriority.Enabled = false;
            //btnSnapToPivot.Image = Properties.Resources.SnapToPivot;
            ct.ResetContour();
            ct.isContourBtnOn = false;
            btnContour.Image = Properties.Resources.ContourOff;
            ct.isContourOn = false;

            btnABDraw.Enabled = false;
            btnCycleLines.Image = Properties.Resources.ABLineCycle;
            //btnCycleLines.Enabled = false;
            btnCycleLinesBk.Image = Properties.Resources.ABLineCycleBk;
            //btnCycleLinesBk.Enabled = false;

            //AutoSteer
            btnAutoSteer.Enabled = false;
            isBtnAutoSteerOn = false;
            btnAutoSteer.Image = Properties.Resources.AutoSteerOff;

            //auto YouTurn shutdown
            yt.isYouTurnBtnOn = false;
            btnAutoYouTurn.Image = Properties.Resources.YouTurnNo;

            btnABDraw.Visible = false;

            yt.ResetYouTurn();
            DisableYouTurnButtons();

            //reset acre and distance counters
            fd.workedAreaTotal = 0;

            //reset GUI areas
            fd.UpdateFieldBoundaryGUIAreas();

            displayFieldName = gStr.gsNone;

            recPath.recList?.Clear();
            recPath.shortestDubinsList?.Clear();
            recPath.shuttleDubinsList?.Clear();

            isPanelBottomHidden = false;

            PanelsAndOGLSize();
            SetZoom();
            worldGrid.isGeoMap = false;
            worldGrid.isRateMap = false;

            panelSim.Top = Height - 60;

            PanelUpdateRightAndBottom();

            using (Bitmap bitmap = Properties.Resources.z_bingMap)
            {
                GL.GenTextures(1, out texture[(int)FormGPS.textures.bingGrid]);
                GL.BindTexture(TextureTarget.Texture2D, texture[(int)FormGPS.textures.bingGrid]);
                BitmapData bitmapData = bitmap.LockBits(new Rectangle(0, 0, bitmap.Width, bitmap.Height), ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
                GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba, bitmapData.Width, bitmapData.Height, 0, OpenTK.Graphics.OpenGL.PixelFormat.Bgra, PixelType.UnsignedByte, bitmapData.Scan0);
                bitmap.UnlockBits(bitmapData);
                GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, 9729);
                GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, 9729);
            }
        }

        public void FieldMenuButtonEnableDisable(bool isOn)
        {
            SmoothABtoolStripMenu.Enabled = isOn;
            deleteContourPathsToolStripMenuItem.Enabled = isOn;
            boundaryToolToolStripMenu.Enabled = isOn;
            offsetFixToolStrip.Enabled = isOn;
            allSettingsMenuItem.Enabled = isOn;

            boundariesToolStripMenuItem.Enabled = isOn;
            headlandToolStripMenuItem.Enabled = isOn;
            headlandBuildToolStripMenuItem.Enabled = isOn;
            flagByLatLonToolStripMenuItem.Enabled = isOn;
            tramLinesMenuField.Enabled = isOn;
            recordedPathStripMenu.Enabled = isOn;
        }

        //take the distance from object and convert to camera data
        public void SetZoom()
        {
            //match grid to cam distance and redo perspective

            camera.gridZoom = camera.camSetDistance / -15;

            gridToolSpacing = (int)(camera.gridZoom / tool.width + 0.5);
            if (gridToolSpacing < 1) gridToolSpacing = 1;

            camera.gridZoom = gridToolSpacing * tool.width;

            oglMain.MakeCurrent();
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();
            Matrix4 mat = Matrix4.CreatePerspectiveFieldOfView((float)fovy, oglMain.AspectRatio, 1f, (float)(camDistanceFactor * camera.camSetDistance));
            GL.LoadMatrix(ref mat);
            GL.MatrixMode(MatrixMode.Modelview);
        }

        //All the files that need to be saved when closing field or app
        //an error log called by all try catches
        public void WriteErrorLog(string strErrorText)
        {
            try
            {
                //set up file and folder if it doesn't exist
                const string strFileName = "Error Log.txt";
                //string strPath = Application.StartupPath;

                //Write out the error appending to existing
                File.AppendAllText(baseDirectory + "\\" + strFileName, strErrorText + " - " +
                    DateTime.Now.ToString() + "\r\n\r\n");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error in WriteErrorLog: " + ex.Message, "Error Logging", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
            }
        }

        //message box pops up with info then goes away
        public void TimedMessageBox(int timeout, string s1, string s2)
        {
            FormTimedMessage form = new FormTimedMessage(timeout, s1, s2);
            form.Show(this);
        }

        public void YesMessageBox(string s1)
        {
            var form = new FormYes(s1);
            form.ShowDialog(this);
        }

        // Generates a random number within a range.
        public double RandomNumber(double min, double max)
        {
            return min + _random.NextDouble() * (max - min);
        }

        private readonly Random _random = new Random();

        private void panelRight_Paint(object sender, PaintEventArgs e)
        {

        }

        private void lblGone_Click(object sender, EventArgs e)
        {
            lblGoneDisdanceClick = !lblGoneDisdanceClick;
        }

        private void lblGoneDisdance_Click(object sender, EventArgs e)
        {
            fd.distanceUser = 0;
        }

        private void toolStripBtnFieldTools_Click_1(object sender, EventArgs e)
        {

        }

        private void btnBuildTracks_small_Click(object sender, EventArgs e)
        {
            if (!isContourPattern)
            {
                btnBuildTracks_small.Image = Properties.Resources.Splitlines_On;
                isContourPattern = true;
                toolContourMenu();
            }
            else
            {
                btnBuildTracks_small.Image = Properties.Resources.Splitlines;
                FileDeletePatternTracks();
                isContourPattern = false;
            }
        }

        private void btnBuildTracks_small1_Click(object sender, EventArgs e)
        {
            btnBuildTracks_nbutton();
        }

        private void toolStripDropDownButton4_Click(object sender, EventArgs e)
        {

        }

        private void lblNumCu_Click(object sender, EventArgs e)
        {

        }

        private void lblCurveLineName_Click(object sender, EventArgs e)
        {
            if (trk.idx > -1)
                SC_MarkLinebyHand();
            else
            {
                TimedMessageBox(3000, "choose a line  ", "  first ");
            }
        }

        private void lblDistanzLine1_Click(object sender, EventArgs e)
        {

        }

        private void lblDistanzLine2_Click(object sender, EventArgs e)
        {

        }

        private void allSettingsMenuItem_Click(object sender, EventArgs e)
        {
            Form form = new FormAllSettings(this);
            form.Show(this);
        }

        // Returns 1 if the lines intersect, otherwis
        public double iE = 0, iN = 0;

        private static readonly double crossingpointEast = 0;
        private static readonly double crossingpointNorth = 0;
        public vec3 Crossingpoint1 = new vec3(crossingpointEast, crossingpointNorth, 0);
        public vec3 Crossingpoint2 = new vec3(crossingpointEast, crossingpointNorth, 0);
        private double CrossingpointEast, CrossingpointNorth;
        private double CrossingpointEast1, CrossingpointNorth1;
        private double CrossingpointEast2, CrossingpointNorth2;
        double DistancetoBoundary;
        double DistancefromBoundary;
        double ABLinelength = 0;
        private bool isHeadingToBoundaryA = true;
        double Curvelength, Findpivotpointdistance = 6000, Findpivotpointdistancenear;
        double curvepoints;
        int countbyPivotAxel = 0;

        int tracksinUse;
        int tracksinUseOld = -100;

        private void CalculateABLength()
        {
            if (trk.idx > -1)
            {
                if (trk.gArr[trk.idx].mode == (int)TrackMode.AB)
                {
                    if (ABLine.isHeadingSameWay)
                    {
                        isHeadingToBoundaryA = true;
                    }
                    else
                    {
                        isHeadingToBoundaryA = false;
                    }

                    tracksinUse = (int)ABLine.howManyPathsAway;
                }
                else
                if (trk.gArr[trk.idx].mode == (int)TrackMode.Curve)
                {
                    if (!curve.isHeadingSameWay)
                    {
                        isHeadingToBoundaryA = false;
                    }
                    else
                    {
                        isHeadingToBoundaryA = true;
                    }
                    tracksinUse = (int)curve.howManyPathsAway;
                }

                if ((bnd.bndList.Count < 1) || (trk.gArr.Count < 1))
                {
                    lblDistancetoBoundary.Visible = false;
                    lblDistancefromBoundary.Visible = false;
                    return;
                }
                else
                {
                    lblDistancetoBoundary.Visible = true;
                    lblDistancefromBoundary.Visible = true;
                }

                if ((tracksinUseOld != tracksinUse) || (tracksinUseOld == -100))
                {
                    tracksinUseOld = tracksinUse;

                    int isStart = 0;

                    if (trk.gArr[trk.idx].mode == (int)TrackMode.AB)
                    {
                        for (int i = 0; i < bnd.bndList[0].fenceLine.Count - 2; i++)
                        {
                            int res = GetLineIntersection(
                            bnd.bndList[0].fenceLine[i].easting,
                            bnd.bndList[0].fenceLine[i].northing,
                            bnd.bndList[0].fenceLine[i + 1].easting,
                            bnd.bndList[0].fenceLine[i + 1].northing,

                            ABLine.currentLinePtA.easting,
                            ABLine.currentLinePtA.northing,
                            ABLine.currentLinePtB.easting,
                            ABLine.currentLinePtB.northing,
                            ref iE, ref iN);

                            if (res == 1)
                            {
                                if (isStart == 0)
                                {
                                    CrossingpointEast1 = CrossingpointEast;
                                    CrossingpointNorth1 = CrossingpointNorth;
                                    Crossingpoint1.easting = CrossingpointEast1;
                                    Crossingpoint1.northing = CrossingpointNorth1;
                                }

                                isStart++;

                                if (isStart == 2)
                                {
                                    CrossingpointEast2 = CrossingpointEast;
                                    CrossingpointNorth2 = CrossingpointNorth;
                                    Crossingpoint2.easting = CrossingpointEast2;
                                    Crossingpoint2.northing = CrossingpointNorth2;
                                }

                            }
                        }
                        double distanceAtoCrossing1 = glm.Distance(Crossingpoint1, ABLine.currentLinePtA);
                        double distanceAtoCrossing2 = glm.Distance(Crossingpoint2, ABLine.currentLinePtA);
                        if (distanceAtoCrossing1 > distanceAtoCrossing2)
                        {
                            (Crossingpoint1, Crossingpoint2) = (Crossingpoint2, Crossingpoint1);
                        }
                    }
                    else  // for curve
                    {
                        bool BeginPoint = false;
                        for (int k = 0; k < curve.curList.Count - 1; k++)
                        {
                            for (int i = 0; i < bnd.bndList[0].fenceLine.Count - 2; i++)
                            {

                                int res = GetLineIntersection(
                            curve.curList[k].easting,
                            curve.curList[k].northing,
                            curve.curList[k + 1].easting,
                            curve.curList[k + 1].northing,

                            bnd.bndList[0].fenceLine[i].easting,
                            bnd.bndList[0].fenceLine[i].northing,
                            bnd.bndList[0].fenceLine[i + 1].easting,
                            bnd.bndList[0].fenceLine[i + 1].northing,

                             ref iE, ref iN);

                                if (res == 1)
                                {
                                    if (isStart == 0)
                                    {
                                        curve.curListDistance.Clear();
                                        BeginPoint = true;
                                        CrossingpointEast1 = CrossingpointEast;
                                        CrossingpointNorth1 = CrossingpointNorth;
                                        Crossingpoint1.easting = CrossingpointEast1;
                                        Crossingpoint1.northing = CrossingpointNorth1;
                                        curve.curListDistance.Add(Crossingpoint1);
                                    }

                                    isStart++;

                                    if (isStart == 2)
                                    {
                                        BeginPoint = false;
                                        CrossingpointEast2 = CrossingpointEast;
                                        CrossingpointNorth2 = CrossingpointNorth;
                                        Crossingpoint2.easting = CrossingpointEast2;
                                        Crossingpoint2.northing = CrossingpointNorth2;
                                        curve.curListDistance.Add(Crossingpoint2);
                                    }

                                }
                            }
                            if (BeginPoint) curve.curListDistance.Add(curve.curList[k]);
                        }
                        if (curve.curList.Count > 3)
                        {
                            double distanceAtoCrossing3 = glm.Distance(Crossingpoint1, curve.curList[0]);
                            double distanceAtoCrossing4 = glm.Distance(Crossingpoint2, curve.curList[0]);
                            if (distanceAtoCrossing3 > distanceAtoCrossing4)
                            {
                                (Crossingpoint1, Crossingpoint2) = (Crossingpoint2, Crossingpoint1);
                            }
                        }
                    }
                    Curvelength = 0;
                    for (int kcurve = 0; kcurve < curve.curListDistance.Count - 2; kcurve++)
                    {
                        Curvelength += glm.Distance(curve.curListDistance[kcurve], curve.curListDistance[kcurve + 1]);
                    }
                    curvepoints = curve.curListDistance.Count;
                }

                DistancetoBoundary = 0;
                Findpivotpointdistancenear = 5000;
                countbyPivotAxel = 0;

                if (trk.gArr[trk.idx].mode == (int)TrackMode.AB)
                {
                    DistancetoBoundary = glm.Distance(Crossingpoint1, pivotAxlePos);
                    DistancefromBoundary = glm.Distance(Crossingpoint2, pivotAxlePos);
                    ABLinelength = glm.Distance(Crossingpoint1, Crossingpoint2);
                }
                else if (trk.gArr[trk.idx].mode == (int)TrackMode.Curve)
                {
                    for (int kcurve = 0; kcurve < curvepoints - 2; kcurve++)
                    {
                        Findpivotpointdistance = glm.Distance(pivotAxlePos, curve.curListDistance[kcurve]);

                        if (Findpivotpointdistancenear > Findpivotpointdistance)
                        {
                            countbyPivotAxel++;
                            Findpivotpointdistancenear = Findpivotpointdistance;
                        }

                    }
                    for (int kc = 0; kc < countbyPivotAxel - 1; kc++)
                    {
                        DistancetoBoundary += glm.Distance(curve.curListDistance[kc], curve.curListDistance[kc + 1]);
                    }
                    DistancefromBoundary = Curvelength - DistancetoBoundary;
                    ABLinelength = Curvelength;

                }


                if (ABLinelength < DistancetoBoundary) DistancefromBoundary *= -1;
                else
                if (ABLinelength < DistancefromBoundary) DistancetoBoundary *= -1;

                if (isHeadingToBoundaryA)
                    (DistancetoBoundary, DistancefromBoundary) = (DistancefromBoundary, DistancetoBoundary);

                if (isMetric)
                {
                    lblDistancetoBoundary.Text = ("Dist to fence : " + DistancetoBoundary.ToString("000.00") + "m ");
                    lblDistancefromBoundary.Text = ("Dist from fence : " + DistancefromBoundary.ToString("000.00") + "m ");
                    if (lblGoneDisdanceClick)
                    {
                        lblGone.Font = new System.Drawing.Font("Tahoma", 22F, System.Drawing.FontStyle.Bold);
                        lblGoneDisdance.Font = new System.Drawing.Font("Tahoma", 22F, System.Drawing.FontStyle.Bold);
                    }
                    else
                    {
                        lblGoneDisdance.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold);
                        lblGone.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold);
                    }
                    lblGone.Text = ("Way");
                    lblGoneDisdance.Text = (": " + fd.DistanceUserMeters + "m ");
                }
                else
                {
                    DistancetoBoundary *= 2.47105;
                    DistancefromBoundary *= 2.47105;
                    lblDistancetoBoundary.Text = ("Dist to fence : " + DistancetoBoundary.ToString("000.00") + "ft ");
                    lblDistancefromBoundary.Text = ("Dist from fence : " + DistancefromBoundary.ToString("000.00") + "ft ");
                    if (lblGoneDisdanceClick)
                    {
                        lblGone.Font = new System.Drawing.Font("Tahoma", 22F, System.Drawing.FontStyle.Bold);
                        lblGoneDisdance.Font = new System.Drawing.Font("Tahoma", 22F, System.Drawing.FontStyle.Bold);
                    }
                    else
                    {
                        lblGoneDisdance.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold);
                        lblGone.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold);
                    }
                    lblGone.Text = ("Way");
                    lblGoneDisdance.Text = (": " + fd.DistanceUserMeters + "ft ");
                }

            }
        }

        private int GetLineIntersection(double p0x, double p0y, double p1x, double p1y,
                double p2x, double p2y, double p3x, double p3y, ref double iEast, ref double iNorth)
        {
            CrossingpointEast = 0;
            CrossingpointNorth = 0;

            double s1x, s1y, s2x, s2y;
            s1x = p1x - p0x;
            s1y = p1y - p0y;

            s2x = p3x - p2x;
            s2y = p3y - p2y;

            double s, t;
            s = (-s1y * (p0x - p2x) + s1x * (p0y - p2y)) / (-s2x * s1y + s1x * s2y);

            if (s >= 0 && s <= 1)
            {
                //check oher side
                t = (s2x * (p0y - p2y) - s2y * (p0x - p2x)) / (-s2x * s1y + s1x * s2y);
                if (t >= 0 && t <= 1)
                {
                    // Collision detected
                    iEast = p0x + (t * s1x);
                    iNorth = p0y + (t * s1y);

                    CrossingpointEast = iEast;
                    CrossingpointNorth = iNorth;
                    return 1;
                }

            }

            return 0; // No collision
        }

        private void lblDistancetoBoundary_Click(object sender, EventArgs e)
        {

        }

        private void lblDistancefromBoundary_Click(object sender, EventArgs e)
        {

        }




    }//class FormGPS
}//namespace AgOpenGPS


