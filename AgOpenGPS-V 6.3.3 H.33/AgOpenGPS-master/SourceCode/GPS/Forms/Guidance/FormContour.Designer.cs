
namespace AgOpenGPS
{
    partial class FormContour
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.oglSelfCont = new OpenTK.GLControl();
            this.timerContour = new System.Windows.Forms.Timer(this.components);
            this.tlp1Cont = new System.Windows.Forms.TableLayoutPanel();
            this.btnExtentPlus = new System.Windows.Forms.Button();
            this.btnExtentMinus = new System.Windows.Forms.Button();
            this.label7 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.btnZoomPlus = new System.Windows.Forms.Button();
            this.btnZoomMinus = new System.Windows.Forms.Button();
            this.lblNewWorkWidth = new System.Windows.Forms.Label();
            this.lblMaxWorkwidth = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.nudSetDistanceLineB = new AgOpenGPS.NudlessNumericUpDown();
            this.nudSetDistanceLineA = new AgOpenGPS.NudlessNumericUpDown();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.btnSaveContourPattern = new System.Windows.Forms.Button();
            this.lblNameCurveB = new System.Windows.Forms.Label();
            this.lblNameCurveA = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.lblDistanceB = new System.Windows.Forms.Label();
            this.lblDisB = new System.Windows.Forms.Label();
            this.lblDistanceA = new System.Windows.Forms.Label();
            this.lblDisA = new System.Windows.Forms.Label();
            this.lblCountLinesAB = new System.Windows.Forms.Label();
            this.lblCountLines = new System.Windows.Forms.Label();
            this.btnMoreLines = new System.Windows.Forms.Button();
            this.btnLessLines = new System.Windows.Forms.Button();
            this.lblMore = new System.Windows.Forms.Label();
            this.btnCycleForwardSecondAB = new System.Windows.Forms.Button();
            this.btnCycleBackwardSecondAB = new System.Windows.Forms.Button();
            this.lblSecondLine = new System.Windows.Forms.Label();
            this.lblFirstABLine = new System.Windows.Forms.Label();
            this.btnleavenosave = new System.Windows.Forms.Button();
            this.btnDeletePattern = new System.Windows.Forms.Button();
            this.lblToolWidth = new System.Windows.Forms.Label();
            this.btnCycleBackwardFirstAB = new System.Windows.Forms.Button();
            this.btnCycleForwardFirstAB = new System.Windows.Forms.Button();
            this.tlp1Cont.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudSetDistanceLineB)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudSetDistanceLineA)).BeginInit();
            this.SuspendLayout();
            // 
            // oglSelfCont
            // 
            this.oglSelfCont.BackColor = System.Drawing.Color.Black;
            this.oglSelfCont.Cursor = System.Windows.Forms.Cursors.Cross;
            this.oglSelfCont.Location = new System.Drawing.Point(0, 1);
            this.oglSelfCont.Margin = new System.Windows.Forms.Padding(0);
            this.oglSelfCont.Name = "oglSelfCont";
            this.oglSelfCont.Size = new System.Drawing.Size(700, 700);
            this.oglSelfCont.TabIndex = 184;
            this.oglSelfCont.VSync = false;
            this.oglSelfCont.Load += new System.EventHandler(this.oglSelfCont_Load);
            this.oglSelfCont.Paint += new System.Windows.Forms.PaintEventHandler(this.oglSelfCont_Paint);
            this.oglSelfCont.MouseDown += new System.Windows.Forms.MouseEventHandler(this.oglSelfCont_MouseDown);
            this.oglSelfCont.Resize += new System.EventHandler(this.oglSelfCont_Resize);
            // 
            // timerContour
            // 
            this.timerContour.Enabled = true;
            this.timerContour.Interval = 500;
            this.timerContour.Tick += new System.EventHandler(this.timerContour_Tick);
            // 
            // tlp1Cont
            // 
            this.tlp1Cont.Anchor = System.Windows.Forms.AnchorStyles.Top;
            this.tlp1Cont.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.tlp1Cont.ColumnCount = 6;
            this.tlp1Cont.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.81615F));
            this.tlp1Cont.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.1435F));
            this.tlp1Cont.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.76009F));
            this.tlp1Cont.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.76009F));
            this.tlp1Cont.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.76009F));
            this.tlp1Cont.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.76009F));
            this.tlp1Cont.Controls.Add(this.btnExtentPlus, 3, 16);
            this.tlp1Cont.Controls.Add(this.btnExtentMinus, 2, 16);
            this.tlp1Cont.Controls.Add(this.label7, 0, 16);
            this.tlp1Cont.Controls.Add(this.label6, 0, 15);
            this.tlp1Cont.Controls.Add(this.btnZoomPlus, 3, 15);
            this.tlp1Cont.Controls.Add(this.btnZoomMinus, 2, 15);
            this.tlp1Cont.Controls.Add(this.lblNewWorkWidth, 2, 10);
            this.tlp1Cont.Controls.Add(this.lblMaxWorkwidth, 0, 10);
            this.tlp1Cont.Controls.Add(this.label5, 0, 9);
            this.tlp1Cont.Controls.Add(this.nudSetDistanceLineB, 2, 5);
            this.tlp1Cont.Controls.Add(this.nudSetDistanceLineA, 2, 2);
            this.tlp1Cont.Controls.Add(this.label4, 0, 5);
            this.tlp1Cont.Controls.Add(this.label3, 0, 2);
            this.tlp1Cont.Controls.Add(this.btnSaveContourPattern, 4, 13);
            this.tlp1Cont.Controls.Add(this.lblNameCurveB, 2, 4);
            this.tlp1Cont.Controls.Add(this.lblNameCurveA, 2, 1);
            this.tlp1Cont.Controls.Add(this.label2, 0, 4);
            this.tlp1Cont.Controls.Add(this.label1, 0, 1);
            this.tlp1Cont.Controls.Add(this.lblDistanceB, 3, 12);
            this.tlp1Cont.Controls.Add(this.lblDisB, 0, 12);
            this.tlp1Cont.Controls.Add(this.lblDistanceA, 3, 11);
            this.tlp1Cont.Controls.Add(this.lblDisA, 0, 11);
            this.tlp1Cont.Controls.Add(this.lblCountLinesAB, 3, 8);
            this.tlp1Cont.Controls.Add(this.lblCountLines, 0, 8);
            this.tlp1Cont.Controls.Add(this.btnMoreLines, 4, 7);
            this.tlp1Cont.Controls.Add(this.btnLessLines, 2, 7);
            this.tlp1Cont.Controls.Add(this.lblMore, 0, 7);
            this.tlp1Cont.Controls.Add(this.btnCycleForwardSecondAB, 4, 3);
            this.tlp1Cont.Controls.Add(this.btnCycleBackwardSecondAB, 2, 3);
            this.tlp1Cont.Controls.Add(this.lblSecondLine, 0, 3);
            this.tlp1Cont.Controls.Add(this.lblFirstABLine, 0, 0);
            this.tlp1Cont.Controls.Add(this.btnleavenosave, 0, 13);
            this.tlp1Cont.Controls.Add(this.btnDeletePattern, 2, 13);
            this.tlp1Cont.Controls.Add(this.lblToolWidth, 0, 6);
            this.tlp1Cont.Controls.Add(this.btnCycleBackwardFirstAB, 2, 0);
            this.tlp1Cont.Controls.Add(this.btnCycleForwardFirstAB, 4, 0);
            this.tlp1Cont.Location = new System.Drawing.Point(703, 1);
            this.tlp1Cont.Name = "tlp1Cont";
            this.tlp1Cont.RowCount = 18;
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 8.038663F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 4.019333F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5.586651F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 8.038663F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 4.019333F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5.586651F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 4.823199F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 7.150062F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 6.628649F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5.847642F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5.490745F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5.163639F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5.163639F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 6.271104F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 6.271104F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5.950466F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5.950466F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tlp1Cont.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tlp1Cont.Size = new System.Drawing.Size(299, 700);
            this.tlp1Cont.TabIndex = 566;
            this.tlp1Cont.Paint += new System.Windows.Forms.PaintEventHandler(this.tlp1Contour_Paint);
            // 
            // btnExtentPlus
            // 
            this.btnExtentPlus.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnExtentPlus.BackColor = System.Drawing.Color.Transparent;
            this.btnExtentPlus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.btnExtentPlus.FlatAppearance.BorderColor = System.Drawing.Color.Black;
            this.btnExtentPlus.FlatAppearance.BorderSize = 0;
            this.btnExtentPlus.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnExtentPlus.Font = new System.Drawing.Font("Tahoma", 36F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnExtentPlus.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnExtentPlus.Location = new System.Drawing.Point(151, 634);
            this.btnExtentPlus.Name = "btnExtentPlus";
            this.btnExtentPlus.Size = new System.Drawing.Size(44, 33);
            this.btnExtentPlus.TabIndex = 599;
            this.btnExtentPlus.Text = "+";
            this.btnExtentPlus.UseCompatibleTextRendering = true;
            this.btnExtentPlus.UseVisualStyleBackColor = false;
            this.btnExtentPlus.Click += new System.EventHandler(this.btnExtentPlus_Click);
            // 
            // btnExtentMinus
            // 
            this.btnExtentMinus.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnExtentMinus.BackColor = System.Drawing.Color.Transparent;
            this.btnExtentMinus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.btnExtentMinus.FlatAppearance.BorderColor = System.Drawing.Color.Black;
            this.btnExtentMinus.FlatAppearance.BorderSize = 0;
            this.btnExtentMinus.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnExtentMinus.Font = new System.Drawing.Font("Tahoma", 36F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnExtentMinus.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnExtentMinus.Location = new System.Drawing.Point(101, 634);
            this.btnExtentMinus.Name = "btnExtentMinus";
            this.btnExtentMinus.Size = new System.Drawing.Size(44, 33);
            this.btnExtentMinus.TabIndex = 598;
            this.btnExtentMinus.Text = "-";
            this.btnExtentMinus.UseCompatibleTextRendering = true;
            this.btnExtentMinus.UseVisualStyleBackColor = false;
            this.btnExtentMinus.Click += new System.EventHandler(this.btnExtentMinus_Click);
            // 
            // label7
            // 
            this.label7.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.label7, 2);
            this.label7.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.ForeColor = System.Drawing.Color.Black;
            this.label7.Location = new System.Drawing.Point(3, 631);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(92, 40);
            this.label7.TabIndex = 597;
            this.label7.Text = "Extent Line";
            this.label7.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label6
            // 
            this.label6.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.label6, 2);
            this.label6.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.ForeColor = System.Drawing.Color.Black;
            this.label6.Location = new System.Drawing.Point(3, 594);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(92, 33);
            this.label6.TabIndex = 596;
            this.label6.Text = "Zoom";
            this.label6.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btnZoomPlus
            // 
            this.btnZoomPlus.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnZoomPlus.BackColor = System.Drawing.Color.Transparent;
            this.btnZoomPlus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.btnZoomPlus.FlatAppearance.BorderColor = System.Drawing.Color.Black;
            this.btnZoomPlus.FlatAppearance.BorderSize = 0;
            this.btnZoomPlus.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnZoomPlus.Font = new System.Drawing.Font("Tahoma", 36F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnZoomPlus.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnZoomPlus.Location = new System.Drawing.Point(151, 594);
            this.btnZoomPlus.Name = "btnZoomPlus";
            this.btnZoomPlus.Size = new System.Drawing.Size(44, 33);
            this.btnZoomPlus.TabIndex = 595;
            this.btnZoomPlus.Text = "+";
            this.btnZoomPlus.UseCompatibleTextRendering = true;
            this.btnZoomPlus.UseVisualStyleBackColor = false;
            this.btnZoomPlus.Click += new System.EventHandler(this.btnZoomPlus_Click);
            // 
            // btnZoomMinus
            // 
            this.btnZoomMinus.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnZoomMinus.BackColor = System.Drawing.Color.Transparent;
            this.btnZoomMinus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.btnZoomMinus.FlatAppearance.BorderColor = System.Drawing.Color.Black;
            this.btnZoomMinus.FlatAppearance.BorderSize = 0;
            this.btnZoomMinus.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnZoomMinus.Font = new System.Drawing.Font("Tahoma", 36F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnZoomMinus.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnZoomMinus.Location = new System.Drawing.Point(101, 594);
            this.btnZoomMinus.Name = "btnZoomMinus";
            this.btnZoomMinus.Size = new System.Drawing.Size(44, 33);
            this.btnZoomMinus.TabIndex = 594;
            this.btnZoomMinus.Text = "-";
            this.btnZoomMinus.UseCompatibleTextRendering = true;
            this.btnZoomMinus.UseVisualStyleBackColor = false;
            this.btnZoomMinus.Click += new System.EventHandler(this.btnZoomMinus_Click);
            // 
            // lblNewWorkWidth
            // 
            this.lblNewWorkWidth.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblNewWorkWidth, 2);
            this.lblNewWorkWidth.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold);
            this.lblNewWorkWidth.ForeColor = System.Drawing.Color.Black;
            this.lblNewWorkWidth.Location = new System.Drawing.Point(102, 401);
            this.lblNewWorkWidth.Name = "lblNewWorkWidth";
            this.lblNewWorkWidth.Size = new System.Drawing.Size(92, 35);
            this.lblNewWorkWidth.TabIndex = 592;
            this.lblNewWorkWidth.Text = "New Workwidth";
            this.lblNewWorkWidth.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblMaxWorkwidth
            // 
            this.lblMaxWorkwidth.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblMaxWorkwidth, 2);
            this.lblMaxWorkwidth.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold);
            this.lblMaxWorkwidth.ForeColor = System.Drawing.Color.Black;
            this.lblMaxWorkwidth.Location = new System.Drawing.Point(3, 401);
            this.lblMaxWorkwidth.Name = "lblMaxWorkwidth";
            this.lblMaxWorkwidth.Size = new System.Drawing.Size(92, 35);
            this.lblMaxWorkwidth.TabIndex = 591;
            this.lblMaxWorkwidth.Text = "Max Workwidth";
            this.lblMaxWorkwidth.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label5
            // 
            this.label5.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.label5, 4);
            this.label5.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold);
            this.label5.ForeColor = System.Drawing.Color.Black;
            this.label5.Location = new System.Drawing.Point(3, 362);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(191, 37);
            this.label5.TabIndex = 590;
            this.label5.Text = "Max   Workwidth  new";
            this.label5.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            // 
            // nudSetDistanceLineB
            // 
            this.nudSetDistanceLineB.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.nudSetDistanceLineB.BackColor = System.Drawing.Color.White;
            this.tlp1Cont.SetColumnSpan(this.nudSetDistanceLineB, 2);
            this.nudSetDistanceLineB.DecimalPlaces = 1;
            this.nudSetDistanceLineB.Font = new System.Drawing.Font("Tahoma", 16F, System.Drawing.FontStyle.Bold);
            this.nudSetDistanceLineB.Location = new System.Drawing.Point(101, 202);
            this.nudSetDistanceLineB.Maximum = new decimal(new int[] {
            200,
            0,
            0,
            0});
            this.nudSetDistanceLineB.Minimum = new decimal(new int[] {
            200,
            0,
            0,
            -2147483648});
            this.nudSetDistanceLineB.Name = "nudSetDistanceLineB";
            this.nudSetDistanceLineB.ReadOnly = true;
            this.nudSetDistanceLineB.Size = new System.Drawing.Size(93, 33);
            this.nudSetDistanceLineB.TabIndex = 589;
            this.nudSetDistanceLineB.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.nudSetDistanceLineB.Click += new System.EventHandler(this.nudSetDistanceLineB_Click);
            // 
            // nudSetDistanceLineA
            // 
            this.nudSetDistanceLineA.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.nudSetDistanceLineA.BackColor = System.Drawing.Color.White;
            this.tlp1Cont.SetColumnSpan(this.nudSetDistanceLineA, 2);
            this.nudSetDistanceLineA.DecimalPlaces = 1;
            this.nudSetDistanceLineA.Font = new System.Drawing.Font("Tahoma", 16F, System.Drawing.FontStyle.Bold);
            this.nudSetDistanceLineA.Location = new System.Drawing.Point(101, 84);
            this.nudSetDistanceLineA.Maximum = new decimal(new int[] {
            200,
            0,
            0,
            0});
            this.nudSetDistanceLineA.Minimum = new decimal(new int[] {
            200,
            0,
            0,
            -2147483648});
            this.nudSetDistanceLineA.Name = "nudSetDistanceLineA";
            this.nudSetDistanceLineA.ReadOnly = true;
            this.nudSetDistanceLineA.Size = new System.Drawing.Size(93, 33);
            this.nudSetDistanceLineA.TabIndex = 588;
            this.nudSetDistanceLineA.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.nudSetDistanceLineA.Click += new System.EventHandler(this.nudSetDistanceLineA_Click);
            // 
            // label4
            // 
            this.label4.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.label4, 2);
            this.label4.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label4.ForeColor = System.Drawing.Color.Black;
            this.label4.Location = new System.Drawing.Point(3, 203);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(92, 28);
            this.label4.TabIndex = 587;
            this.label4.Text = "Move B";
            this.label4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label3
            // 
            this.label3.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.label3, 2);
            this.label3.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.ForeColor = System.Drawing.Color.Black;
            this.label3.Location = new System.Drawing.Point(3, 85);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(92, 28);
            this.label3.TabIndex = 586;
            this.label3.Text = "Move A";
            this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btnSaveContourPattern
            // 
            this.btnSaveContourPattern.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnSaveContourPattern.BackColor = System.Drawing.Color.Transparent;
            this.btnSaveContourPattern.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1Cont.SetColumnSpan(this.btnSaveContourPattern, 2);
            this.btnSaveContourPattern.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnSaveContourPattern.FlatAppearance.BorderSize = 0;
            this.btnSaveContourPattern.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnSaveContourPattern.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnSaveContourPattern.Image = global::AgOpenGPS.Properties.Resources.ContourLines;
            this.btnSaveContourPattern.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnSaveContourPattern.Location = new System.Drawing.Point(214, 513);
            this.btnSaveContourPattern.Name = "btnSaveContourPattern";
            this.tlp1Cont.SetRowSpan(this.btnSaveContourPattern, 2);
            this.btnSaveContourPattern.Size = new System.Drawing.Size(69, 71);
            this.btnSaveContourPattern.TabIndex = 585;
            this.btnSaveContourPattern.UseVisualStyleBackColor = false;
            this.btnSaveContourPattern.Click += new System.EventHandler(this.btnSaveContourPattern_Click);
            // 
            // lblNameCurveB
            // 
            this.lblNameCurveB.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblNameCurveB, 4);
            this.lblNameCurveB.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblNameCurveB.ForeColor = System.Drawing.Color.Black;
            this.lblNameCurveB.Location = new System.Drawing.Point(103, 172);
            this.lblNameCurveB.Name = "lblNameCurveB";
            this.lblNameCurveB.Size = new System.Drawing.Size(191, 26);
            this.lblNameCurveB.TabIndex = 584;
            this.lblNameCurveB.Text = "Field";
            this.lblNameCurveB.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblNameCurveA
            // 
            this.lblNameCurveA.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblNameCurveA, 4);
            this.lblNameCurveA.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblNameCurveA.ForeColor = System.Drawing.Color.Black;
            this.lblNameCurveA.Location = new System.Drawing.Point(103, 54);
            this.lblNameCurveA.Name = "lblNameCurveA";
            this.lblNameCurveA.Size = new System.Drawing.Size(191, 26);
            this.lblNameCurveA.TabIndex = 583;
            this.lblNameCurveA.Text = "Field";
            this.lblNameCurveA.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label2
            // 
            this.label2.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.label2, 2);
            this.label2.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.ForeColor = System.Drawing.Color.Black;
            this.label2.Location = new System.Drawing.Point(3, 172);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(92, 26);
            this.label2.TabIndex = 582;
            this.label2.Text = "Curve B";
            this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label1
            // 
            this.label1.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.label1, 2);
            this.label1.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.Color.Black;
            this.label1.Location = new System.Drawing.Point(3, 54);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(92, 26);
            this.label1.TabIndex = 581;
            this.label1.Text = "Curve A";
            this.label1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblDistanceB
            // 
            this.lblDistanceB.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblDistanceB, 3);
            this.lblDistanceB.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblDistanceB.ForeColor = System.Drawing.Color.Black;
            this.lblDistanceB.Location = new System.Drawing.Point(164, 473);
            this.lblDistanceB.Name = "lblDistanceB";
            this.lblDistanceB.Size = new System.Drawing.Size(119, 33);
            this.lblDistanceB.TabIndex = 580;
            this.lblDistanceB.Text = "Dist A2B2";
            this.lblDistanceB.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblDisB
            // 
            this.lblDisB.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblDisB, 3);
            this.lblDisB.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblDisB.ForeColor = System.Drawing.Color.Black;
            this.lblDisB.Location = new System.Drawing.Point(14, 473);
            this.lblDisB.Name = "lblDisB";
            this.lblDisB.Size = new System.Drawing.Size(119, 33);
            this.lblDisB.TabIndex = 579;
            this.lblDisB.Text = "Distance B";
            this.lblDisB.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblDistanceA
            // 
            this.lblDistanceA.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblDistanceA, 3);
            this.lblDistanceA.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblDistanceA.ForeColor = System.Drawing.Color.Black;
            this.lblDistanceA.Location = new System.Drawing.Point(164, 438);
            this.lblDistanceA.Name = "lblDistanceA";
            this.lblDistanceA.Size = new System.Drawing.Size(119, 33);
            this.lblDistanceA.TabIndex = 578;
            this.lblDistanceA.Text = "Dist A1B1";
            this.lblDistanceA.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblDisA
            // 
            this.lblDisA.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblDisA, 3);
            this.lblDisA.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblDisA.ForeColor = System.Drawing.Color.Black;
            this.lblDisA.Location = new System.Drawing.Point(14, 438);
            this.lblDisA.Name = "lblDisA";
            this.lblDisA.Size = new System.Drawing.Size(119, 33);
            this.lblDisA.TabIndex = 577;
            this.lblDisA.Text = "Distance A";
            this.lblDisA.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblCountLinesAB
            // 
            this.lblCountLinesAB.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblCountLinesAB, 3);
            this.lblCountLinesAB.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblCountLinesAB.ForeColor = System.Drawing.Color.Black;
            this.lblCountLinesAB.Location = new System.Drawing.Point(177, 317);
            this.lblCountLinesAB.Name = "lblCountLinesAB";
            this.lblCountLinesAB.Size = new System.Drawing.Size(92, 43);
            this.lblCountLinesAB.TabIndex = 576;
            this.lblCountLinesAB.Text = "Count Lines";
            this.lblCountLinesAB.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblCountLines
            // 
            this.lblCountLines.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblCountLines, 3);
            this.lblCountLines.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblCountLines.ForeColor = System.Drawing.Color.Black;
            this.lblCountLines.Location = new System.Drawing.Point(28, 317);
            this.lblCountLines.Name = "lblCountLines";
            this.lblCountLines.Size = new System.Drawing.Size(92, 43);
            this.lblCountLines.TabIndex = 575;
            this.lblCountLines.Text = "Count Lines";
            this.lblCountLines.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btnMoreLines
            // 
            this.btnMoreLines.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnMoreLines.BackColor = System.Drawing.Color.Transparent;
            this.btnMoreLines.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1Cont.SetColumnSpan(this.btnMoreLines, 2);
            this.btnMoreLines.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnMoreLines.FlatAppearance.BorderSize = 0;
            this.btnMoreLines.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnMoreLines.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnMoreLines.Image = global::AgOpenGPS.Properties.Resources.ABLineCycle;
            this.btnMoreLines.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnMoreLines.Location = new System.Drawing.Point(217, 272);
            this.btnMoreLines.Name = "btnMoreLines";
            this.btnMoreLines.Size = new System.Drawing.Size(63, 40);
            this.btnMoreLines.TabIndex = 573;
            this.btnMoreLines.UseVisualStyleBackColor = false;
            this.btnMoreLines.Click += new System.EventHandler(this.btnMoreLines_Click);
            // 
            // btnLessLines
            // 
            this.btnLessLines.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnLessLines.BackColor = System.Drawing.Color.Transparent;
            this.btnLessLines.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1Cont.SetColumnSpan(this.btnLessLines, 2);
            this.btnLessLines.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnLessLines.FlatAppearance.BorderSize = 0;
            this.btnLessLines.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnLessLines.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnLessLines.Image = global::AgOpenGPS.Properties.Resources.ABLineCycleBk;
            this.btnLessLines.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnLessLines.Location = new System.Drawing.Point(116, 272);
            this.btnLessLines.Name = "btnLessLines";
            this.btnLessLines.Size = new System.Drawing.Size(63, 40);
            this.btnLessLines.TabIndex = 572;
            this.btnLessLines.UseVisualStyleBackColor = false;
            this.btnLessLines.Click += new System.EventHandler(this.btnLessLines_Click);
            // 
            // lblMore
            // 
            this.lblMore.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblMore, 2);
            this.lblMore.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblMore.ForeColor = System.Drawing.Color.Black;
            this.lblMore.Location = new System.Drawing.Point(3, 269);
            this.lblMore.Name = "lblMore";
            this.lblMore.Size = new System.Drawing.Size(92, 46);
            this.lblMore.TabIndex = 571;
            this.lblMore.Text = "More Lines";
            this.lblMore.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btnCycleForwardSecondAB
            // 
            this.btnCycleForwardSecondAB.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnCycleForwardSecondAB.BackColor = System.Drawing.Color.Transparent;
            this.btnCycleForwardSecondAB.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1Cont.SetColumnSpan(this.btnCycleForwardSecondAB, 2);
            this.btnCycleForwardSecondAB.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnCycleForwardSecondAB.FlatAppearance.BorderSize = 0;
            this.btnCycleForwardSecondAB.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnCycleForwardSecondAB.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnCycleForwardSecondAB.Image = global::AgOpenGPS.Properties.Resources.ABLineCycle;
            this.btnCycleForwardSecondAB.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnCycleForwardSecondAB.Location = new System.Drawing.Point(217, 122);
            this.btnCycleForwardSecondAB.Name = "btnCycleForwardSecondAB";
            this.btnCycleForwardSecondAB.Size = new System.Drawing.Size(63, 46);
            this.btnCycleForwardSecondAB.TabIndex = 569;
            this.btnCycleForwardSecondAB.UseVisualStyleBackColor = false;
            this.btnCycleForwardSecondAB.Click += new System.EventHandler(this.btnCycleForwardSecondAB_Click);
            // 
            // btnCycleBackwardSecondAB
            // 
            this.btnCycleBackwardSecondAB.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnCycleBackwardSecondAB.BackColor = System.Drawing.Color.Transparent;
            this.btnCycleBackwardSecondAB.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1Cont.SetColumnSpan(this.btnCycleBackwardSecondAB, 2);
            this.btnCycleBackwardSecondAB.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnCycleBackwardSecondAB.FlatAppearance.BorderSize = 0;
            this.btnCycleBackwardSecondAB.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnCycleBackwardSecondAB.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnCycleBackwardSecondAB.Image = global::AgOpenGPS.Properties.Resources.ABLineCycleBk;
            this.btnCycleBackwardSecondAB.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnCycleBackwardSecondAB.Location = new System.Drawing.Point(116, 122);
            this.btnCycleBackwardSecondAB.Name = "btnCycleBackwardSecondAB";
            this.btnCycleBackwardSecondAB.Size = new System.Drawing.Size(63, 46);
            this.btnCycleBackwardSecondAB.TabIndex = 568;
            this.btnCycleBackwardSecondAB.UseVisualStyleBackColor = false;
            this.btnCycleBackwardSecondAB.Click += new System.EventHandler(this.btnCycleBackwardSecondAB_Click);
            // 
            // lblSecondLine
            // 
            this.lblSecondLine.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.lblSecondLine.BackColor = System.Drawing.Color.Red;
            this.tlp1Cont.SetColumnSpan(this.lblSecondLine, 2);
            this.lblSecondLine.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblSecondLine.ForeColor = System.Drawing.Color.Black;
            this.lblSecondLine.Location = new System.Drawing.Point(3, 119);
            this.lblSecondLine.Name = "lblSecondLine";
            this.lblSecondLine.Size = new System.Drawing.Size(92, 52);
            this.lblSecondLine.TabIndex = 567;
            this.lblSecondLine.Text = "Second AB ";
            this.lblSecondLine.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblFirstABLine
            // 
            this.lblFirstABLine.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.lblFirstABLine.BackColor = System.Drawing.Color.LimeGreen;
            this.tlp1Cont.SetColumnSpan(this.lblFirstABLine, 2);
            this.lblFirstABLine.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblFirstABLine.ForeColor = System.Drawing.Color.Black;
            this.lblFirstABLine.Location = new System.Drawing.Point(3, 1);
            this.lblFirstABLine.Name = "lblFirstABLine";
            this.lblFirstABLine.Size = new System.Drawing.Size(92, 52);
            this.lblFirstABLine.TabIndex = 566;
            this.lblFirstABLine.Text = "First AB ";
            this.lblFirstABLine.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btnleavenosave
            // 
            this.btnleavenosave.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnleavenosave.BackColor = System.Drawing.Color.Transparent;
            this.btnleavenosave.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.tlp1Cont.SetColumnSpan(this.btnleavenosave, 2);
            this.btnleavenosave.FlatAppearance.BorderColor = System.Drawing.Color.Black;
            this.btnleavenosave.FlatAppearance.BorderSize = 0;
            this.btnleavenosave.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnleavenosave.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnleavenosave.Image = global::AgOpenGPS.Properties.Resources.SwitchOff;
            this.btnleavenosave.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnleavenosave.Location = new System.Drawing.Point(17, 514);
            this.btnleavenosave.Name = "btnleavenosave";
            this.tlp1Cont.SetRowSpan(this.btnleavenosave, 2);
            this.btnleavenosave.Size = new System.Drawing.Size(63, 70);
            this.btnleavenosave.TabIndex = 519;
            this.btnleavenosave.UseVisualStyleBackColor = false;
            this.btnleavenosave.Click += new System.EventHandler(this.btnleavenosave_Click);
            // 
            // btnDeletePattern
            // 
            this.btnDeletePattern.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnDeletePattern.BackColor = System.Drawing.Color.Transparent;
            this.btnDeletePattern.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1Cont.SetColumnSpan(this.btnDeletePattern, 2);
            this.btnDeletePattern.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnDeletePattern.FlatAppearance.BorderSize = 0;
            this.btnDeletePattern.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnDeletePattern.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnDeletePattern.Image = global::AgOpenGPS.Properties.Resources.Trash;
            this.btnDeletePattern.ImageAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnDeletePattern.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnDeletePattern.Location = new System.Drawing.Point(101, 510);
            this.btnDeletePattern.Name = "btnDeletePattern";
            this.tlp1Cont.SetRowSpan(this.btnDeletePattern, 2);
            this.btnDeletePattern.Size = new System.Drawing.Size(93, 78);
            this.btnDeletePattern.TabIndex = 6;
            this.btnDeletePattern.Text = "Pattern";
            this.btnDeletePattern.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.btnDeletePattern.UseVisualStyleBackColor = false;
            this.btnDeletePattern.Click += new System.EventHandler(this.btnDeletePattern_Click);
            // 
            // lblToolWidth
            // 
            this.lblToolWidth.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1Cont.SetColumnSpan(this.lblToolWidth, 6);
            this.lblToolWidth.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblToolWidth.ForeColor = System.Drawing.Color.Black;
            this.lblToolWidth.Location = new System.Drawing.Point(12, 236);
            this.lblToolWidth.Name = "lblToolWidth";
            this.lblToolWidth.Size = new System.Drawing.Size(275, 31);
            this.lblToolWidth.TabIndex = 511;
            this.lblToolWidth.Text = "3.86";
            this.lblToolWidth.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btnCycleBackwardFirstAB
            // 
            this.btnCycleBackwardFirstAB.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnCycleBackwardFirstAB.BackColor = System.Drawing.Color.Transparent;
            this.btnCycleBackwardFirstAB.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1Cont.SetColumnSpan(this.btnCycleBackwardFirstAB, 2);
            this.btnCycleBackwardFirstAB.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnCycleBackwardFirstAB.FlatAppearance.BorderSize = 0;
            this.btnCycleBackwardFirstAB.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnCycleBackwardFirstAB.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnCycleBackwardFirstAB.Image = global::AgOpenGPS.Properties.Resources.ABLineCycleBk;
            this.btnCycleBackwardFirstAB.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnCycleBackwardFirstAB.Location = new System.Drawing.Point(116, 4);
            this.btnCycleBackwardFirstAB.Name = "btnCycleBackwardFirstAB";
            this.btnCycleBackwardFirstAB.Size = new System.Drawing.Size(63, 46);
            this.btnCycleBackwardFirstAB.TabIndex = 507;
            this.btnCycleBackwardFirstAB.UseVisualStyleBackColor = false;
            this.btnCycleBackwardFirstAB.Click += new System.EventHandler(this.btnCycleBackwardFirstAB_Click);
            // 
            // btnCycleForwardFirstAB
            // 
            this.btnCycleForwardFirstAB.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnCycleForwardFirstAB.BackColor = System.Drawing.Color.Transparent;
            this.btnCycleForwardFirstAB.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1Cont.SetColumnSpan(this.btnCycleForwardFirstAB, 2);
            this.btnCycleForwardFirstAB.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnCycleForwardFirstAB.FlatAppearance.BorderSize = 0;
            this.btnCycleForwardFirstAB.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnCycleForwardFirstAB.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnCycleForwardFirstAB.Image = global::AgOpenGPS.Properties.Resources.ABLineCycle;
            this.btnCycleForwardFirstAB.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnCycleForwardFirstAB.Location = new System.Drawing.Point(217, 4);
            this.btnCycleForwardFirstAB.Name = "btnCycleForwardFirstAB";
            this.btnCycleForwardFirstAB.Size = new System.Drawing.Size(63, 46);
            this.btnCycleForwardFirstAB.TabIndex = 5;
            this.btnCycleForwardFirstAB.UseVisualStyleBackColor = false;
            this.btnCycleForwardFirstAB.Click += new System.EventHandler(this.btnCycleForwardFirstAB_Click);
            // 
            // FormContour
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.WhiteSmoke;
            this.ClientSize = new System.Drawing.Size(1006, 726);
            this.ControlBox = false;
            this.Controls.Add(this.tlp1Cont);
            this.Controls.Add(this.oglSelfCont);
            this.ForeColor = System.Drawing.Color.Black;
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.MinimumSize = new System.Drawing.Size(1022, 673);
            this.Name = "FormContour";
            this.ShowInTaskbar = false;
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterParent;
            this.Text = "FormContour";
            this.Load += new System.EventHandler(this.FormContour_Load);
            this.tlp1Cont.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.nudSetDistanceLineB)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudSetDistanceLineA)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private OpenTK.GLControl oglSelfCont;
        private System.Windows.Forms.Timer timerContour;
        private System.Windows.Forms.TableLayoutPanel tlp1Cont;
        private System.Windows.Forms.Button btnCycleForwardSecondAB;
        private System.Windows.Forms.Button btnCycleBackwardSecondAB;
        private System.Windows.Forms.Label lblSecondLine;
        private System.Windows.Forms.Label lblFirstABLine;
        private System.Windows.Forms.Button btnleavenosave;
        private System.Windows.Forms.Button btnDeletePattern;
        private System.Windows.Forms.Label lblToolWidth;
        private System.Windows.Forms.Button btnCycleBackwardFirstAB;
        private System.Windows.Forms.Button btnCycleForwardFirstAB;
        private System.Windows.Forms.Button btnMoreLines;
        private System.Windows.Forms.Button btnLessLines;
        private System.Windows.Forms.Label lblMore;
        private System.Windows.Forms.Label lblDistanceB;
        private System.Windows.Forms.Label lblDisB;
        private System.Windows.Forms.Label lblDistanceA;
        private System.Windows.Forms.Label lblDisA;
        private System.Windows.Forms.Label lblCountLinesAB;
        private System.Windows.Forms.Label lblCountLines;
        private System.Windows.Forms.Label lblNameCurveB;
        private System.Windows.Forms.Label lblNameCurveA;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button btnSaveContourPattern;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private NudlessNumericUpDown nudSetDistanceLineB;
        private NudlessNumericUpDown nudSetDistanceLineA;
        private System.Windows.Forms.Label lblMaxWorkwidth;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label lblNewWorkWidth;
        private System.Windows.Forms.Button btnZoomPlus;
        private System.Windows.Forms.Button btnZoomMinus;
        private System.Windows.Forms.Button btnExtentMinus;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Button btnExtentPlus;
    }
}