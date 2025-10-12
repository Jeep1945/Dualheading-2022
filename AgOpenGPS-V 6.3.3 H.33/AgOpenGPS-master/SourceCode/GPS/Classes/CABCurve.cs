using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace AgOpenGPS
{
    public class CABCurve
    {
        //pointers to mainform controls
        private readonly FormGPS mf;

        public bool isBtnCurveOn, isCurveSet, isOkToAddDesPoints;

        //flag for starting stop adding points
        public bool isBtnTrackOn, isMakingCurve;

        public double distanceFromCurrentLinePivot;
        public double distanceFromRefLine;
        public double distanceFromRefLinePattern;

        public bool isHeadingSameWay = true;

        public double abHeading, abLength;   //#############################################
        public int lineWidth, CurveroundPoints;

        public double howManyPathsAway, lastHowManyPathsAway;
        public int howManyPathsAwayiCL = 0;
        public vec2 refPoint1 = new vec2(1, 1), refPoint2 = new vec2(2, 2);

        private int A, B, C;
        private int rA, rB;
        private int rAPa, rBPa;

        public int currentLocationIndex;

        //pure pursuit values
        public vec2 goalPointCu = new vec2(0, 0);

        public vec2 radiusPointCu = new vec2(0, 0);
        public double steerAngleCu, rEastCu, rNorthCu, ppRadiusCu, manualUturnHeading;

        public bool isSmoothWindowOpen, isLooping;
        public List<vec3> smooList = new List<vec3>();

        //the list of points of curve to drive on
        public List<vec3> curList = new List<vec3>();
        public List<vec3> curListDistance = new List<vec3>();

        bool isReady = false, isBusyWorking = false;

        //the list of points of curve new list from async
        public List<vec3> newCurList = new List<vec3>();

        //the current curve reference line.
        //public CTrk refCurve = new CTrk();

        public List<CCurveLines> curveArr = new List<CCurveLines>();
        public int numCurveLines, numCurveLineSelected;

        public bool isCurveValid, isLateralTriggered;

        public double lastSecond = 0;

        public List<vec3> desList = new List<vec3>();
        public string desName = "**", nam = "";

        public double pivotDistanceError, pivotDistanceErrorLast, pivotDerivative, pivotDerivativeSmoothed, lastCurveDistance = 10000;

        //derivative counters
        private int counter2;

        public CTrk track = new CTrk();
        double DistancePivotAxleA;
        double DistancePivotAxleB;

        public double inty;

        public CABCurve(FormGPS _f)
        {
            //constructor
            mf = _f;
            desList.Capacity = 1024;
            curList.Capacity = 1024;
            lineWidth = Properties.Settings.Default.setDisplay_lineWidth;

        }

        public void BuildCurveCurrentList(vec3 pivot)
        {
            double minDistA = 1000000, minDistB;
            double minDistAPa = 1000000, minDistBPa;
            int ccPaLineA = 0;
            int ccPaLineA1 = 0;

            //move the ABLine over based on the overlap amount set in vehicle
            double widthMinusOverlap = mf.tool.width - mf.tool.overlap;

            int idx = mf.trk.idx;

            track = mf.trk.gArr[mf.trk.idx];

            nam = track.name;

            if (track.mode != (int)TrackMode.waterPivot)
            {

                int refCount = track.curvePts.Count;
                if (refCount < 5)
                {
                    curList?.Clear();
                    return;
                }

                //close call hit
                int cc = 0, dd;

                for (int j = 0; j < refCount; j += 10)
                {
                    double dist = ((mf.guidanceLookPos.easting - track.curvePts[j].easting)
                        * (mf.guidanceLookPos.easting - track.curvePts[j].easting))
                                    + ((mf.guidanceLookPos.northing - track.curvePts[j].northing)
                                    * (mf.guidanceLookPos.northing - track.curvePts[j].northing));
                    if (dist < minDistA)
                    {
                        minDistA = dist;
                        cc = j;
                    }
                }

                minDistA = minDistB = 1000000;

                dd = cc + 7; if (dd > refCount - 1) dd = refCount;
                cc -= 7; if (cc < 0) cc = 0;

                //find the closest 2 points to current close call
                for (int j = cc; j < dd; j++)
                {
                    double dist = ((mf.guidanceLookPos.easting - track.curvePts[j].easting)
                        * (mf.guidanceLookPos.easting - track.curvePts[j].easting))
                                    + ((mf.guidanceLookPos.northing - track.curvePts[j].northing)
                                    * (mf.guidanceLookPos.northing - track.curvePts[j].northing));
                    if (dist < minDistA)
                    {
                        minDistB = minDistA;
                        rB = rA;
                        minDistA = dist;
                        rA = j;
                    }
                    else if (dist < minDistB)
                    {
                        minDistB = dist;
                        rB = j;
                    }
                }

                //reset the line over jump
                isLateralTriggered = false;

                if (rA > rB) { C = rA; rA = rB; rB = C; }

                //same way as line creation or not
                isHeadingSameWay = Math.PI - Math.Abs(Math.Abs(pivot.heading - track.curvePts[rA].heading) - Math.PI) < glm.PIBy2;

                if (mf.yt.isYouTurnTriggered && !mf.yt.isGoingStraightThrough) isHeadingSameWay = !isHeadingSameWay;

                //which side of the closest point are we on is next
                //calculate endpoints of reference line based on closest point
                refPoint1.easting = track.curvePts[rA].easting - (Math.Sin(track.curvePts[rA].heading) * 300.0);
                refPoint1.northing = track.curvePts[rA].northing - (Math.Cos(track.curvePts[rA].heading) * 300.0);

                refPoint2.easting = track.curvePts[rA].easting + (Math.Sin(track.curvePts[rA].heading) * 300.0);
                refPoint2.northing = track.curvePts[rA].northing + (Math.Cos(track.curvePts[rA].heading) * 300.0);

                if ((nam.Length > 4 && nam.Substring(0, 5) == "&FIX ") || (nam.Length > 3 && nam.Substring(0, 4) == "&Pa "))
                {
                    track.nudgeDistance = 0;
                }

                if (idx > -1 && track.nudgeDistance != 0)
                {
                    refPoint1.easting += (Math.Sin(track.curvePts[rA].heading + glm.PIBy2) * track.nudgeDistance);
                    refPoint1.northing += (Math.Cos(track.curvePts[rA].heading + glm.PIBy2) * track.nudgeDistance);

                    refPoint2.easting += (Math.Sin(track.curvePts[rA].heading + glm.PIBy2) * track.nudgeDistance);
                    refPoint2.northing += (Math.Cos(track.curvePts[rA].heading + glm.PIBy2) * track.nudgeDistance);
                }

                //x2-x1
                double dx = refPoint2.easting - refPoint1.easting;
                //z2-z1
                double dz = refPoint2.northing - refPoint1.northing;

                widthMinusOverlap = mf.tool.width - mf.tool.overlap;

                //how far are we away from the reference line at 90 degrees - 2D cross product and distance
                distanceFromRefLine = ((dz * mf.guidanceLookPos.easting) - (dx * mf.guidanceLookPos.northing) + (refPoint2.easting
                                    * refPoint1.northing) - (refPoint2.northing * refPoint1.easting))
                                    / Math.Sqrt((dz * dz) + (dx * dx));

                distanceFromRefLine -= (0.5 * widthMinusOverlap);

                double RefDist = (distanceFromRefLine + (isHeadingSameWay ? mf.tool.offset : -mf.tool.offset)) / widthMinusOverlap;

                if (RefDist < 0) howManyPathsAway = (int)(RefDist - 0.5);
                else howManyPathsAway = (int)(RefDist + 0.5);

                if (track.mode != (int)TrackMode.bndCurve)
                {
                    //build current list
                    isCurveValid = true;

                    if ((howManyPathsAway == lastHowManyPathsAway) && (nam.Substring(0, 1) != "&"))
                    {
                        return;
                    }

                    lastHowManyPathsAway = howManyPathsAway;

                    //build the current line
                    curList?.Clear();

                    double distAway = widthMinusOverlap * howManyPathsAway + (isHeadingSameWay ? -mf.tool.offset : mf.tool.offset) + track.nudgeDistance;

                    if (nam.Length > 4 && nam.Substring(0, 5) == "&FIX ")
                    {
                        distAway = 0;
                        mf.isSideGuideLines = false;
                    }
                    else
                    {
                        distAway += (0.5 * widthMinusOverlap);
                        mf.isSideGuideLines = true;
                        if (howManyPathsAway > -1) howManyPathsAway += 1;
                    }


                    double step = widthMinusOverlap * 0.48;
                    if (step > 4) step = 4;
                    if (step < 1) step = 1;

                    double distSqAway = (distAway * distAway) - 0.01;
                    bool Add;
                    vec3 point = new vec3();
                    //  #######################################################

                    double PatterntoolwidthA, PatterntoolwidthB;
                    if (nam.Length > 3 && nam.Substring(0, 4) == "&Pa ")
                    {

                        // to look if on side A or B
                        PatterntoolwidthA = mf.DistanceA1B1 / mf.howManyPathPattern;
                        PatterntoolwidthB = mf.DistanceA2B2 / mf.howManyPathPattern;
                        double guidanceLookDistPa1;
                        DistancePivotAxleA = glm.Distance(mf.pivotAxlePos, mf.ct.ContourLineList[0][0]);
                        DistancePivotAxleB = glm.Distance(mf.pivotAxlePos, mf.ct.ContourLineList[0][mf.ct.ContourLineList[0].Count - 1]);
                        if (DistancePivotAxleA < DistancePivotAxleB)
                            guidanceLookDistPa1 = (Math.Max(PatterntoolwidthA * 0.5, mf.avgSpeed * 0.277777 * mf.guidanceLookAheadTime));
                        else guidanceLookDistPa1 = (Math.Max(PatterntoolwidthB * 0.5, mf.avgSpeed * 0.277777 * mf.guidanceLookAheadTime));
                        mf.guidanceLookPos.easting = mf.pivotAxlePos.easting + (Math.Sin(mf.fixHeading) * guidanceLookDistPa1);
                        mf.guidanceLookPos.northing = mf.pivotAxlePos.northing + (Math.Cos(mf.fixHeading) * guidanceLookDistPa1);

                        int refCountPaLine = mf.ct.ContourLineList.Count;
                        int refCountPaPoints = mf.ct.ContourLineList[0].Count;

                        //close call hit
                        int ccPa = 0, ddPa, ddPaLineA;
                        int maxrefCountPaLine = 5, maxrefCountPaPoint;
                        if (refCountPaPoints < 42) maxrefCountPaPoint = 2;
                        else maxrefCountPaPoint = 10;
                        //##################################

                        for (int ij = 0; ij < (refCountPaLine); ij += maxrefCountPaLine)
                        {
                            for (int j = 0; j < (mf.ct.ContourLineList[ij].Count); j += maxrefCountPaPoint)
                            {
                                double distPa = ((mf.guidanceLookPos.easting - mf.ct.ContourLineList[ij][j].easting)
                                                * (mf.guidanceLookPos.easting - mf.ct.ContourLineList[ij][j].easting))
                                                + ((mf.guidanceLookPos.northing - mf.ct.ContourLineList[ij][j].northing)
                                                * (mf.guidanceLookPos.northing - mf.ct.ContourLineList[ij][j].northing));
                                if (distPa < minDistAPa)
                                {
                                    minDistAPa = distPa;
                                    ccPa = j;
                                    ccPaLineA = ij;
                                }
                                //Console.WriteLine("minDistAPa 1: " + j + "  " + ij + "  " + distPa);
                            }
                        }
                        //Console.WriteLine("ccPa: " + ccPa + "  ccPaLineA  " + ccPaLineA + "  minDistAPa " + minDistAPa);
                        minDistAPa = minDistBPa = 1000000;

                        ddPa = ccPa + 7; if (ddPa > refCountPaPoints - 1) ddPa = refCountPaPoints;
                        ccPa -= 7; if (ccPa < 0) ccPa = 0;

                        ddPaLineA = ccPaLineA + 5; if (ddPaLineA > refCountPaLine) ccPaLineA = refCountPaLine;
                        ccPaLineA -= 5; if (ccPaLineA < 0) ccPaLineA = 0;

                        //find the closest 2 points to current close call
                        for (int jLine = ccPaLineA; jLine < ddPaLineA; jLine++)
                        {
                            if (mf.ct.ContourLineList[jLine].Count < 2)
                                return;
                            if (ddPa >= mf.ct.ContourLineList[jLine].Count) ddPa = mf.ct.ContourLineList[jLine].Count - 1;

                            for (int j = ccPa; j < ddPa; j++)
                            {
                                double distPa = ((mf.guidanceLookPos.easting - mf.ct.ContourLineList[jLine][j].easting)
                                    * (mf.guidanceLookPos.easting - mf.ct.ContourLineList[jLine][j].easting))
                                                + ((mf.guidanceLookPos.northing - mf.ct.ContourLineList[jLine][j].northing)
                                                * (mf.guidanceLookPos.northing - mf.ct.ContourLineList[jLine][j].northing));
                                if (distPa < minDistAPa)
                                {
                                    minDistBPa = minDistAPa;
                                    rBPa = rAPa;
                                    minDistAPa = distPa;
                                    rAPa = j;
                                    ccPaLineA1 = jLine;
                                }
                                else if (distPa < minDistBPa)
                                {
                                    minDistBPa = distPa;
                                    rBPa = j;
                                }
                            }
                        }
                        //reset the line over jump
                        isLateralTriggered = false;

                        if (rAPa > rBPa) { C = rAPa; rAPa = rBPa; rBPa = C; }

                        //Console.WriteLine("ccPaLineA1   2: " + ccPaLineA1);
                        //Console.WriteLine("ccPaLineB1   2: " + ccPaLineB1);
                        //Console.WriteLine("rBPa   0: " + rBPa);
                        //Console.WriteLine("rAPa   0: " + rAPa);
                        //Console.WriteLine("minDistAPa : " + minDistAPa);
                        //Console.WriteLine("minDistBPa : " + minDistBPa);

                        //same way as line creation or not

                        isHeadingSameWay = Math.PI - Math.Abs(Math.Abs(pivot.heading - mf.ct.ContourLineList[0][rAPa].heading) - Math.PI) < glm.PIBy2;

                        howManyPathsAwayiCL = ccPaLineA1;
                        howManyPathsAway = howManyPathsAwayiCL;
                    }

                    //############################################################################################################
                    //build the current line
                    curList?.Clear();

                    // make the right Line to go
                    if ((nam.Length > 3 && nam.Substring(0, 4) == "&Pa ") && (mf.ct.ContourLineList.Count > howManyPathsAwayiCL))
                    {
                        for (int i = 0; i < mf.ct.ContourLineList[howManyPathsAwayiCL].Count - 1; i++)
                        {
                            if ((howManyPathsAwayiCL > -1) && (mf.ct.ContourLineList[howManyPathsAwayiCL].Count > i))
                            {
                                point = new vec3(mf.ct.ContourLineList[howManyPathsAwayiCL][i].easting,
                                  mf.ct.ContourLineList[howManyPathsAwayiCL][i].northing,
                                  mf.ct.ContourLineList[howManyPathsAwayiCL][i].heading);
                                curList.Add(point);
                            }
                        }
                    }
                    else
                    {
                        for (int i = 0; i < refCount; i++)
                        {
                            point = new vec3(
                                track.curvePts[i].easting + (Math.Sin(glm.PIBy2 + track.curvePts[i].heading) * distAway),
                                track.curvePts[i].northing + (Math.Cos(glm.PIBy2 + track.curvePts[i].heading) * distAway),
                                track.curvePts[i].heading);
                            Add = true;


                            for (int t = 0; t < refCount; t++)
                            {
                                double dist = ((point.easting - track.curvePts[t].easting) * (point.easting - track.curvePts[t].easting))
                                    + ((point.northing - track.curvePts[t].northing) * (point.northing - track.curvePts[t].northing));
                                if (dist < distSqAway)
                                {
                                    Add = false;
                                    break;
                                }
                            }

                            if (Add)
                            {
                                if (curList.Count > 0)
                                {
                                    double dist = ((point.easting - curList[curList.Count - 1].easting) * (point.easting - curList[curList.Count - 1].easting))
                                        + ((point.northing - curList[curList.Count - 1].northing) * (point.northing - curList[curList.Count - 1].northing));
                                    if (dist > step)
                                        curList.Add(point);
                                }
                                else curList.Add(point);
                            }
                        }
                    }

                    int cnt = curList.Count;
                    if (cnt > 6)
                    {
                        vec3[] arr = new vec3[cnt];
                        curList.CopyTo(arr);

                        //for (int i = 1; i < (curList.Count - 1); i++)
                        //{
                        //    arr[i].easting = (curList[i - 1].easting + curList[i].easting + curList[i + 1].easting) / 3;
                        //    arr[i].northing = (curList[i - 1].northing + curList[i].northing + curList[i + 1].northing) / 3;
                        //}
                        curList.Clear();

                        for (int i = 0; i < (arr.Length - 1); i++)
                        {
                            arr[i].heading = Math.Atan2(arr[i + 1].easting - arr[i].easting, arr[i + 1].northing - arr[i].northing);
                            if (arr[i].heading < 0) arr[i].heading += glm.twoPI;
                            if (arr[i].heading >= glm.twoPI) arr[i].heading -= glm.twoPI;
                        }

                        arr[arr.Length - 1].heading = arr[arr.Length - 2].heading;

                        //if (mf.tool.isToolTrailing)
                        //{
                        //    //depending on hitch is different profile of draft
                        //    double hitch;
                        //    if (mf.tool.isToolTBT && mf.tool.tankTrailingHitchLength < 0)
                        //    {
                        //        hitch = mf.tool.tankTrailingHitchLength * 0.65;
                        //        hitch += mf.tool.trailingHitchLength * 0.5;
                        //    }
                        //    else hitch = mf.tool.trailingHitchLength * 1.0;// - mf.vehicle.wheelbase;

                        //    //move the line forward based on hitch length ratio
                        //    for (int i = 0; i < arr.Length; i++)
                        //    {
                        //        arr[i].easting -= Math.Sin(arr[i].heading) * (hitch);
                        //        arr[i].northing -= Math.Cos(arr[i].heading) * (hitch);
                        //    }

                        //    ////average the points over 3, center weighted
                        //    //for (int i = 1; i < arr.Length - 2; i++)
                        //    //{
                        //    //    arr2[i].easting = (arr[i - 1].easting + arr[i].easting + arr[i + 1].easting) / 3;
                        //    //    arr2[i].northing = (arr[i - 1].northing + arr[i].northing + arr[i + 1].northing) / 3;
                        //    //}

                        //    //recalculate the heading
                        //    for (int i = 0; i < (arr.Length - 1); i++)
                        //    {
                        //        arr[i].heading = Math.Atan2(arr[i + 1].easting - arr[i].easting, arr[i + 1].northing - arr[i].northing);
                        //        if (arr[i].heading < 0) arr[i].heading += glm.twoPI;
                        //        if (arr[i].heading >= glm.twoPI) arr[i].heading -= glm.twoPI;
                        //    }

                        //    arr[arr.Length - 1].heading = arr[arr.Length - 2].heading;
                        //}

                        //replace the array
                        //curList.AddRange(arr);
                        cnt = arr.Length;
                        double distance;

                        //add the first point of loop - it will be p1
                        curList.Add(arr[0]);
                        //curList.Add(arr[1]);

                        for (int i = 0; i < cnt - 3; i++)
                        {
                            // add p1
                            curList.Add(arr[i + 1]);

                            distance = glm.Distance(arr[i + 1], arr[i + 2]);

                            if (distance > step)
                            {
                                int loopTimes = (int)(distance / step + 1);
                                for (int j = 1; j < loopTimes; j++)
                                {
                                    vec3 pos = new vec3(glm.Catmull(j / (double)(loopTimes), arr[i], arr[i + 1], arr[i + 2], arr[i + 3]));
                                    curList.Add(pos);
                                }
                            }
                        }

                        curList.Add(arr[cnt - 2]);
                        curList.Add(arr[cnt - 1]);

                        //to calc heading based on next and previous points to give an average heading.
                        cnt = curList.Count;
                        arr = new vec3[cnt];
                        cnt--;
                        curList.CopyTo(arr);
                        curList.Clear();

                        curList.Add(new vec3(arr[0]));

                        //middle points
                        for (int i = 1; i < cnt; i++)
                        {
                            vec3 pt3 = new vec3(arr[i]);
                            pt3.heading = Math.Atan2(arr[i + 1].easting - arr[i - 1].easting, arr[i + 1].northing - arr[i - 1].northing);
                            if (pt3.heading < 0) pt3.heading += glm.twoPI;
                            curList.Add(pt3);
                        }

                        int k = arr.Length - 1;
                        vec3 pt33 = new vec3(arr[k]);
                        pt33.heading = Math.Atan2(arr[k].easting - arr[k - 1].easting, arr[k].northing - arr[k - 1].northing);
                        if (pt33.heading < 0) pt33.heading += glm.twoPI;
                        curList.Add(pt33);

                        if (mf.trk.gArr == null || mf.trk.gArr.Count == 0 || idx == -1) return;

                        if (mf.bnd.bndList.Count > 0 && !(track.mode == (int)TrackMode.bndCurve))
                        {
                            int ptCnt = curList.Count - 1;

                            bool isAdding = false;
                            //end
                            while (mf.bnd.bndList[0].fenceLineEar.IsPointInPolygon(curList[curList.Count - 1]))
                            {
                                isAdding = true;
                                for (int i = 1; i < 10; i++)
                                {
                                    vec3 pt = new vec3(curList[ptCnt]);
                                    pt.easting += (Math.Sin(pt.heading) * i * 2);
                                    pt.northing += (Math.Cos(pt.heading) * i * 2);
                                    curList.Add(pt);
                                }
                                ptCnt = curList.Count - 1;
                            }

                            if (isAdding)
                            {
                                vec3 pt = new vec3(curList[curList.Count - 1]);
                                for (int i = 1; i < 5; i++)
                                {
                                    pt.easting += (Math.Sin(pt.heading) * 2);
                                    pt.northing += (Math.Cos(pt.heading) * 2);
                                    curList.Add(pt);
                                }
                            }

                            isAdding = false;

                            //and the beginning
                            pt33 = new vec3(curList[0]);

                            while (mf.bnd.bndList[0].fenceLineEar.IsPointInPolygon(curList[0]))
                            {
                                isAdding = true;
                                pt33 = new vec3(curList[0]);

                                for (int i = 1; i < 10; i++)
                                {
                                    vec3 pt = new vec3(pt33);
                                    pt.easting -= (Math.Sin(pt.heading) * i * 2);
                                    pt.northing -= (Math.Cos(pt.heading) * i * 2);
                                    curList.Insert(0, pt);
                                }
                            }

                            if (isAdding)
                            {
                                vec3 pt = new vec3(curList[0]);
                                for (int i = 1; i < 5; i++)
                                {
                                    pt.easting -= (Math.Sin(pt.heading) * 2);
                                    pt.northing -= (Math.Cos(pt.heading) * 2);
                                    curList.Insert(0, pt);
                                }
                            }

                        }
                    }
                }
                else
                {
                    //is boundary curve - use task
                    if (isReady)
                    {
                        curList = new List<vec3>(newCurList);
                        isReady = false;
                    }

                    //build current list
                    isCurveValid = true;

                    if (howManyPathsAway == lastHowManyPathsAway)
                    {
                        return;
                    }

                    lastHowManyPathsAway = howManyPathsAway;

                    //build the current line
                    //curList?.Clear();

                    double distAway = (mf.tool.width - mf.tool.overlap) * howManyPathsAway + (isHeadingSameWay ? -mf.tool.offset : mf.tool.offset) + mf.trk.gArr[idx].nudgeDistance;

                    if (howManyPathsAway > -1) howManyPathsAway += 1;

                    distAway += (0.5 * (mf.tool.width - mf.tool.overlap));

                    if (!isBusyWorking) _ = BuildNewCurveAsync(distAway, refCount, track);

                }
            }
            else //pivot guide list
            {
                //pivot circle center
                refPoint1 = track.ptA;

                //cross product
                isHeadingSameWay = ((mf.pivotAxlePos.easting - refPoint1.easting) * (mf.steerAxlePos.northing - refPoint1.northing)
                    - (mf.pivotAxlePos.northing - refPoint1.northing) * (mf.steerAxlePos.easting - refPoint1.easting)) < 0;

                //how far are we away from the reference line at 90 degrees - 2D cross product and distance
                distanceFromRefLine = glm.Distance(mf.pivotAxlePos, refPoint1);

                distanceFromRefLine -= (0.5 * widthMinusOverlap);

                double RefDist = (distanceFromRefLine
                    + (isHeadingSameWay ? mf.tool.offset : -mf.tool.offset)
                    + track.nudgeDistance) / widthMinusOverlap;

                if (RefDist < 0) howManyPathsAway = (int)(RefDist - 0.5);
                else howManyPathsAway = (int)(RefDist + 0.5);

                //build current list
                isCurveValid = true;

                //build the current line
                curList?.Clear();

                double distAway = widthMinusOverlap * howManyPathsAway
                    + (isHeadingSameWay ? -mf.tool.offset : mf.tool.offset) - track.nudgeDistance;

                distAway += (0.5 * widthMinusOverlap);

                if (howManyPathsAway > -1) howManyPathsAway += 1;

                double pointSpacing = distAway * 0.05;

                //distAway += mf.trk.gArr[idx].nudgeDistance;

                vec3 currentPos = new vec3(refPoint1.easting - distAway, refPoint1.northing, 0);

                while (currentPos.heading < glm.twoPI)
                {
                    //Update the position of the car
                    currentPos.easting += pointSpacing * Math.Sin(currentPos.heading);
                    currentPos.northing += pointSpacing * Math.Cos(currentPos.heading);

                    //Update the heading
                    currentPos.heading += (pointSpacing / distAway);

                    //Add the new coordinate to the path
                    curList.Add(currentPos);
                }

                vec3[] arr = new vec3[curList.Count];
                curList.CopyTo(arr);
                curList.Clear();

                for (int i = 0; i < (arr.Length - 1); i++)
                {
                    arr[i].heading = Math.Atan2(arr[i + 1].easting - arr[i].easting, arr[i + 1].northing - arr[i].northing);
                    if (arr[i].heading < 0) arr[i].heading += glm.twoPI;
                    if (arr[i].heading >= glm.twoPI) arr[i].heading -= glm.twoPI;
                }

                arr[arr.Length - 1].heading = arr[arr.Length - 2].heading;

                curList.AddRange(arr);
            }

            lastSecond = mf.secondsSinceStart;
        }

        public async Task BuildNewCurveAsync(double distAway, int refCount, CTrk track)
        {
            await Task.Run(() =>
            {
                isBusyWorking = true;
                isReady = false;

                newCurList?.Clear();

                double step = (mf.tool.width - mf.tool.overlap) * 0.48;
                if (step > 4) step = 4;
                if (step < 1) step = 1;

                double distSqAway = (distAway * distAway) - 0.01;

                vec3 point;

                try
                {

                    for (int i = 0; i < refCount; i++)
                    {
                        point = new vec3(
                        track.curvePts[i].easting + (Math.Sin(glm.PIBy2 + track.curvePts[i].heading) * distAway),
                        track.curvePts[i].northing + (Math.Cos(glm.PIBy2 + track.curvePts[i].heading) * distAway),
                        track.curvePts[i].heading);
                        bool Add = true;

                        for (int t = 0; t < refCount; t++)
                        {
                            double dist = ((point.easting - track.curvePts[t].easting) * (point.easting - track.curvePts[t].easting))
                                + ((point.northing - track.curvePts[t].northing) * (point.northing - track.curvePts[t].northing));
                            if (dist < distSqAway)
                            {
                                Add = false;
                                break;
                            }
                        }

                        if (Add)
                        {
                            if (newCurList.Count > 0)
                            {
                                double dist = ((point.easting - newCurList[newCurList.Count - 1].easting) * (point.easting - newCurList[newCurList.Count - 1].easting))
                                    + ((point.northing - newCurList[newCurList.Count - 1].northing) * (point.northing - newCurList[newCurList.Count - 1].northing));
                                if (dist > step)
                                    newCurList.Add(point);
                            }
                            else newCurList.Add(point);
                        }
                    }

                    int cnt = newCurList.Count;
                    if (cnt > 6)
                    {
                        vec3[] arr = new vec3[cnt];
                        newCurList.CopyTo(arr);

                        newCurList.Clear();

                        for (int i = 0; i < (arr.Length - 1); i++)
                        {
                            arr[i].heading = Math.Atan2(arr[i + 1].easting - arr[i].easting, arr[i + 1].northing - arr[i].northing);
                            if (arr[i].heading < 0) arr[i].heading += glm.twoPI;
                            if (arr[i].heading >= glm.twoPI) arr[i].heading -= glm.twoPI;
                        }

                        arr[arr.Length - 1].heading = arr[arr.Length - 2].heading;

                        cnt = arr.Length;
                        double distance;

                        //add the first point of loop - it will be p1
                        newCurList.Add(arr[0]);
                        //newCurList.Add(arr[1]);

                        for (int i = 0; i < cnt - 3; i++)
                        {
                            // add p1
                            newCurList.Add(arr[i + 1]);

                            distance = glm.Distance(arr[i + 1], arr[i + 2]);

                            if (distance > step)
                            {
                                int loopTimes = (int)(distance / step + 1);
                                for (int j = 1; j < loopTimes; j++)
                                {
                                    vec3 pos = new vec3(glm.Catmull(j / (double)(loopTimes), arr[i], arr[i + 1], arr[i + 2], arr[i + 3]));
                                    newCurList.Add(pos);
                                }
                            }
                        }

                        newCurList.Add(arr[cnt - 2]);
                        newCurList.Add(arr[cnt - 1]);

                        //to calc heading based on next and previous points to give an average heading.
                        cnt = newCurList.Count;
                        arr = new vec3[cnt];
                        cnt--;
                        newCurList.CopyTo(arr);
                        newCurList.Clear();

                        newCurList.Add(new vec3(arr[0]));

                        //middle points
                        for (int i = 1; i < cnt; i++)
                        {
                            vec3 pt3 = new vec3(arr[i]);
                            pt3.heading = Math.Atan2(arr[i + 1].easting - arr[i - 1].easting, arr[i + 1].northing - arr[i - 1].northing);
                            if (pt3.heading < 0) pt3.heading += glm.twoPI;
                            newCurList.Add(pt3);
                        }

                        int k = arr.Length - 1;
                        vec3 pt33 = new vec3(arr[k]);
                        pt33.heading = Math.Atan2(arr[k].easting - arr[k - 1].easting, arr[k].northing - arr[k - 1].northing);
                        if (pt33.heading < 0) pt33.heading += glm.twoPI;
                        newCurList.Add(pt33);

                        if (track == null || track.curvePts.Count == 0)
                        {
                            isReady = false;
                            isBusyWorking = false;
                            return;
                        }

                        if (mf.bnd.bndList.Count > 0 && !(track.mode == (int)TrackMode.bndCurve))
                        {
                            int ptCnt = newCurList.Count - 1;

                            bool isAdding = false;
                            //end
                            while (mf.bnd.bndList[0].fenceLineEar.IsPointInPolygon(newCurList[newCurList.Count - 1]))
                            {
                                isAdding = true;
                                for (int i = 1; i < 10; i++)
                                {
                                    vec3 pt = new vec3(newCurList[ptCnt]);
                                    pt.easting += (Math.Sin(pt.heading) * i * 2);
                                    pt.northing += (Math.Cos(pt.heading) * i * 2);
                                    newCurList.Add(pt);
                                }
                                ptCnt = newCurList.Count - 1;
                            }

                            if (isAdding)
                            {
                                vec3 pt = new vec3(newCurList[newCurList.Count - 1]);
                                for (int i = 1; i < 5; i++)
                                {
                                    pt.easting += (Math.Sin(pt.heading) * 2);
                                    pt.northing += (Math.Cos(pt.heading) * 2);
                                    newCurList.Add(pt);
                                }
                            }

                            isAdding = false;

                            //and the beginning
                            pt33 = new vec3(newCurList[0]);

                            while (mf.bnd.bndList[0].fenceLineEar.IsPointInPolygon(newCurList[0]))
                            {
                                isAdding = true;
                                pt33 = new vec3(newCurList[0]);

                                for (int i = 1; i < 10; i++)
                                {
                                    vec3 pt = new vec3(pt33);
                                    pt.easting -= (Math.Sin(pt.heading) * i * 2);
                                    pt.northing -= (Math.Cos(pt.heading) * i * 2);
                                    newCurList.Insert(0, pt);
                                }
                            }

                            if (isAdding)
                            {
                                vec3 pt = new vec3(newCurList[0]);
                                for (int i = 1; i < 5; i++)
                                {
                                    pt.easting -= (Math.Sin(pt.heading) * 2);
                                    pt.northing -= (Math.Cos(pt.heading) * 2);
                                    newCurList.Insert(0, pt);
                                }
                            }
                        }

                        isReady = true;
                    }

                }
                catch (Exception)
                {
                    //throw;
                }

                isBusyWorking = false;
            });
        }

        public void GetCurrentCurveLine(vec3 pivot, vec3 steer)
        {
            if (mf.trk.gArr[mf.trk.idx].curvePts == null || mf.trk.gArr[mf.trk.idx].curvePts.Count < 5)
            {
                if (mf.trk.gArr[mf.trk.idx].mode != (int)TrackMode.waterPivot)
                {
                    return;
                }
            }

            double dist, dx, dz;
            double minDistA = 1000000, minDistB = 1000000;
            //int ptCount = curList.Count;

            if (curList.Count > 0)
            {
                if (mf.yt.isYouTurnTriggered && mf.yt.DistanceFromYouTurnLine())//do the pure pursuit from youTurn
                {
                    //now substitute what it thinks are AB line values with auto turn values
                    steerAngleCu = mf.yt.steerAngleYT;
                    distanceFromCurrentLinePivot = mf.yt.distanceFromCurrentLine;

                    goalPointCu = mf.yt.goalPointYT;
                    radiusPointCu.easting = mf.yt.radiusPointYT.easting;
                    radiusPointCu.northing = mf.yt.radiusPointYT.northing;
                    ppRadiusCu = mf.yt.ppRadiusYT;
                    mf.vehicle.modeActualXTE = (distanceFromCurrentLinePivot);
                }
                else if (mf.isStanleyUsed)//Stanley
                {
                    mf.gyd.StanleyGuidanceCurve(pivot, steer, ref curList);
                }
                else// Pure Pursuit ------------------------------------------
                {

                    minDistA = double.MaxValue;
                    //close call hit

                    //If is a curve
                    if (mf.trk.gArr[mf.trk.idx].mode <= (int)TrackMode.Curve)
                    {
                        minDistA = minDistB = double.MaxValue;
                        //close call hit
                        int cc = 0, dd;

                        for (int j = 0; j < curList.Count; j += 10)
                        {
                            dist = glm.DistanceSquared(pivot, curList[j]);
                            if (dist < minDistA)
                            {
                                minDistA = dist;
                                cc = j;
                            }
                        }

                        minDistA = double.MaxValue;

                        dd = cc + 8; if (dd > curList.Count - 1) dd = curList.Count;
                        cc -= 8; if (cc < 0) cc = 0;

                        //find the closest 2 points to current close call
                        for (int j = cc; j < dd; j++)
                        {
                            dist = glm.DistanceSquared(pivot, curList[j]);
                            if (dist < minDistA)
                            {
                                minDistB = minDistA;
                                B = A;
                                minDistA = dist;
                                A = j;
                            }
                            else if (dist < minDistB)
                            {
                                minDistB = dist;
                                B = j;
                            }
                        }

                        //just need to make sure the points continue ascending or heading switches all over the place
                        if (A > B) { C = A; A = B; B = C; }

                        currentLocationIndex = A;

                        if (A > curList.Count - 1 || B > curList.Count - 1)
                            return;
                    }
                    else
                    {
                        for (int j = 0; j < curList.Count; j++)
                        {
                            dist = glm.DistanceSquared(pivot, curList[j]);
                            if (dist < minDistA)
                            {
                                minDistA = dist;
                                A = j;
                            }
                        }

                        currentLocationIndex = A;

                        if (A > curList.Count - 1)
                            return;

                        //initial forward Test if pivot InRange AB
                        if (A == curList.Count - 1) B = 0;
                        else B = A + 1;

                        if (glm.InRangeBetweenAB(curList[A].easting, curList[A].northing,
                             curList[B].easting, curList[B].northing, pivot.easting, pivot.northing))
                            goto SegmentFound;

                        A = currentLocationIndex;
                        //step back one
                        if (A == 0)
                        {
                            A = curList.Count - 1;
                            B = 0;
                        }
                        else
                        {
                            A--;
                            B = A + 1;
                        }

                        if (glm.InRangeBetweenAB(curList[A].easting, curList[A].northing,
                            curList[B].easting, curList[B].northing, pivot.easting, pivot.northing))
                            goto SegmentFound;

                        //realy really lost
                        return;
                    }

                    SegmentFound:

                    //get the distance from currently active AB line

                    dx = curList[B].easting - curList[A].easting;
                    dz = curList[B].northing - curList[A].northing;

                    if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dz) < Double.Epsilon) return;

                    //abHeading = Math.Atan2(dz, dx);

                    //how far from current AB Line is fix
                    if (mf.vehicle.purePursuitIntegralGain > 16)
                        distanceFromCurrentLinePivot = ((dz * pivot.easting) - (dx * pivot.northing) + (curList[B].easting
                                   * curList[A].northing) - (curList[B].northing * curList[A].easting))
                                       / Math.Sqrt((dz * dz) + (dx * dx));
                    else distanceFromCurrentLinePivot = ((dz * steer.easting) - (dx * steer.northing) + (curList[B].easting * curList[A].northing) - (curList[B].northing * curList[A].easting))
                                     / Math.Sqrt((dz * dz) + (dx * dx));

                    //integral slider is set to 0
                    if (mf.vehicle.purePursuitIntegralGain != 0 && !mf.isReverse)
                    {
                        pivotDistanceError = distanceFromCurrentLinePivot * 0.2 + pivotDistanceError * 0.8;

                        if (counter2++ > 4)
                        {
                            pivotDerivative = pivotDistanceError - pivotDistanceErrorLast;
                            pivotDistanceErrorLast = pivotDistanceError;
                            counter2 = 0;
                            pivotDerivative *= 2;

                            //limit the derivative
                            //if (pivotDerivative > 0.03) pivotDerivative = 0.03;
                            //if (pivotDerivative < -0.03) pivotDerivative = -0.03;
                            //if (Math.Abs(pivotDerivative) < 0.01) pivotDerivative = 0;
                        }

                        //pivotErrorTotal = pivotDistanceError + pivotDerivative;

                        if (mf.isBtnAutoSteerOn && mf.avgSpeed > 2.5 && Math.Abs(pivotDerivative) < 0.1)
                        {
                            //if over the line heading wrong way, rapidly decrease integral
                            if ((inty < 0 && distanceFromCurrentLinePivot < 0) || (inty > 0 && distanceFromCurrentLinePivot > 0))
                            {
                                inty += pivotDistanceError * mf.vehicle.purePursuitIntegralGain * -0.04;
                            }
                            else
                            {
                                if (Math.Abs(distanceFromCurrentLinePivot) > 0.02)
                                {
                                    inty += pivotDistanceError * mf.vehicle.purePursuitIntegralGain * -0.02;
                                    if (inty > 0.2) inty = 0.2;
                                    else if (inty < -0.2) inty = -0.2;
                                }
                            }
                        }
                        else inty *= 0.95;
                    }
                    else inty = 0;

                    // ** Pure pursuit ** - calc point on ABLine closest to current position
                    double U = (((pivot.easting - curList[A].easting) * dx)
                                + ((pivot.northing - curList[A].northing) * dz))
                                / ((dx * dx) + (dz * dz));

                    rEastCu = curList[A].easting + (U * dx);
                    rNorthCu = curList[A].northing + (U * dz);
                    manualUturnHeading = curList[A].heading;

                    //update base on autosteer settings and distance from line
                    double goalPointDistance = mf.vehicle.UpdateGoalPointDistance();

                    bool ReverseHeading = mf.isReverse ? !isHeadingSameWay : isHeadingSameWay;

                    int count = ReverseHeading ? 1 : -1;
                    vec3 start = new vec3(rEastCu, rNorthCu, 0);
                    double distSoFar = 0;

                    for (int i = ReverseHeading ? B : A; i < curList.Count && i >= 0;)
                    {
                        // used for calculating the length squared of next segment.
                        double tempDist = glm.Distance(start, curList[i]);

                        //will we go too far?
                        if ((tempDist + distSoFar) > goalPointDistance)
                        {
                            double j = (goalPointDistance - distSoFar) / tempDist; // the remainder to yet travel

                            goalPointCu.easting = (((1 - j) * start.easting) + (j * curList[i].easting));
                            goalPointCu.northing = (((1 - j) * start.northing) + (j * curList[i].northing));
                            break;
                        }
                        else distSoFar += tempDist;
                        start = curList[i];
                        i += count;
                        if (i < 0) i = curList.Count - 1;
                        if (i > curList.Count - 1) i = 0;
                    }

                    if (mf.trk.gArr[mf.trk.idx].mode <= (int)TrackMode.Curve)
                    {
                        if (mf.isBtnAutoSteerOn && !mf.isReverse)
                        {
                            if (isHeadingSameWay)
                            {
                                if (glm.Distance(goalPointCu, curList[(curList.Count - 1)]) < 0.5)
                                {
                                    mf.TimedMessageBox(2000, gStr.gsGuidanceStopped, gStr.gsPastEndOfCurve);
                                    mf.btnAutoSteer.PerformClick();
                                }
                            }
                            else
                            {
                                if (glm.Distance(goalPointCu, curList[0]) < 0.5)
                                {
                                    mf.btnAutoSteer.PerformClick();
                                    mf.TimedMessageBox(2000, gStr.gsGuidanceStopped, gStr.gsPastEndOfCurve);
                                }
                            }
                        }
                    }

                    //calc "D" the distance from pivot axle to lookahead point
                    double goalPointDistanceSquared = glm.DistanceSquared(goalPointCu.northing, goalPointCu.easting, pivot.northing, pivot.easting);

                    //calculate the the delta x in local coordinates and steering angle degrees based on wheelbase
                    //double localHeading = glm.twoPI - mf.fixHeading;

                    double localHeading;
                    if (ReverseHeading) localHeading = glm.twoPI - mf.fixHeading + inty;
                    else localHeading = glm.twoPI - mf.fixHeading - inty;

                    ppRadiusCu = goalPointDistanceSquared / (2 * (((goalPointCu.easting - pivot.easting) * Math.Cos(localHeading)) + ((goalPointCu.northing - pivot.northing) * Math.Sin(localHeading))));

                    steerAngleCu = glm.toDegrees(Math.Atan(2 * (((goalPointCu.easting - pivot.easting) * Math.Cos(localHeading))
                        + ((goalPointCu.northing - pivot.northing) * Math.Sin(localHeading))) * mf.vehicle.wheelbase / goalPointDistanceSquared));

                    if (mf.ahrs.imuRoll != 88888)
                        steerAngleCu += mf.ahrs.imuRoll * -mf.gyd.sideHillCompFactor;

                    if (steerAngleCu < -mf.vehicle.maxSteerAngle) steerAngleCu = -mf.vehicle.maxSteerAngle;
                    if (steerAngleCu > mf.vehicle.maxSteerAngle) steerAngleCu = mf.vehicle.maxSteerAngle;

                    if (!isHeadingSameWay)
                        distanceFromCurrentLinePivot *= -1.0;

                    //used for acquire/hold mode
                    mf.vehicle.modeActualXTE = (distanceFromCurrentLinePivot);

                    double steerHeadingError = (pivot.heading - curList[A].heading);
                    //Fix the circular error
                    if (steerHeadingError > Math.PI)
                        steerHeadingError -= Math.PI;
                    else if (steerHeadingError < -Math.PI)
                        steerHeadingError += Math.PI;

                    if (steerHeadingError > glm.PIBy2)
                        steerHeadingError -= Math.PI;
                    else if (steerHeadingError < -glm.PIBy2)
                        steerHeadingError += Math.PI;

                    mf.vehicle.modeActualHeadingError = glm.toDegrees(steerHeadingError);

                    //Convert to centimeters
                    mf.guidanceLineDistanceOff = (short)Math.Round(distanceFromCurrentLinePivot * 1000.0, MidpointRounding.AwayFromZero);
                    mf.guidanceLineSteerAngle = (short)(steerAngleCu * 100);
                }
            }
            else
            {
                //invalid distance so tell AS module
                distanceFromCurrentLinePivot = 32000;
                mf.guidanceLineDistanceOff = 32000;
            }
        }

        public void DrawSlope()
        {

            GL.LineWidth(3);     // 
            GL.Color3(1.0f, 0.0f, 1.0f);   // violett  driving line
            GL.Begin(PrimitiveType.LineStrip);
            for (int iPoint = 0; iPoint < mf.trk.gArr[mf.trk.gArr.Count - 1].curvePts.Count; iPoint++)
            {
                GL.Vertex3(mf.trk.gArr[mf.trk.gArr.Count - 1].curvePts[iPoint].easting, mf.trk.gArr[mf.trk.gArr.Count - 1].curvePts[iPoint].northing, 0);
            }
            GL.Disable(EnableCap.LineStipple);
            GL.End();

            GL.LineWidth(2);
            GL.Color3(0.96, 0.2f, 0.2f);   // red   activ AB curve
            GL.Begin(PrimitiveType.LineStrip);

            if (mf.tooltrk.gToolArr.Count > 1)
            {
                for (int h = 0; h < mf.tooltrk.gToolArr.Count - 1; h++)
                {
                    for (int jh = 0; jh < mf.tooltrk.gToolArr[h].curve_Toolpivot_Pts.Count; jh++)
                    {
                         GL.Vertex3(mf.tooltrk.gToolArr[mf.tooltrk.gToolArr.Count - 1].curve_Toolpivot_Pts[jh].easting, mf.tooltrk.gToolArr[mf.tooltrk.gToolArr.Count - 1].curve_Toolpivot_Pts[jh].northing, 0);
                    }
                }
            }
            GL.End();
        }

        public void DrawContourPatternCurve()
        {
            int Minline = mf.GuidanceLine9, Maxline = mf.GuidanceLine9;
            int ptCount = mf.trk.gArr[mf.trk.idx].curvePts.Count;
            //int ptCountPa, miCL = 4;
            // draws the violett AB buildline
            GL.End();
            GL.LineWidth(1);
            GL.Disable(EnableCap.LineStipple);
            GL.Color3(0.5f, 0.5f, 1.0f);   //blue 

            if (mf.isSideGuideLines)
            {
                Minline = howManyPathsAwayiCL - mf.GuidanceLine9;
                if (Minline < 0) Minline = 0;
                Maxline = howManyPathsAwayiCL + mf.GuidanceLine9 + 1;
                if (Maxline > mf.ct.ContourLineList.Count) Maxline = mf.ct.ContourLineList.Count;
            }


            if ((mf.ct.ContourLineList.Count > 1) && (mf.trk.idx > 1))
            {
                for (int iCL = Minline; iCL < Maxline; iCL++)
                {
                    GL.Color3(0.5f, 0.5f, 1.0f);   //blue 
                    GL.Begin(PrimitiveType.LineStrip);
                    for (int iPoint = 0; iPoint < mf.ct.ContourLineList[iCL].Count; iPoint++)
                    {
                        GL.Vertex3(mf.ct.ContourLineList[iCL][iPoint].easting, mf.ct.ContourLineList[iCL][iPoint].northing, 0);
                    }
                    GL.End();

                    // linenumbers shown
                    if ((mf.isLineNumberon) && (mf.camera.camSetDistance > mf.tool.width * -120))
                    {
                        GL.Color3(0.96, 0.2f, 0.2f);   // red   activ AB curve
                        mf.font.DrawText3D(mf.ct.ContourLineList[iCL][1].easting, mf.ct.ContourLineList[iCL][1].northing, iCL.ToString(), 2);
                        mf.font.DrawText3D(mf.ct.ContourLineList[iCL][(mf.ct.ContourLineList[iCL].Count - 1) / 2].easting, mf.ct.ContourLineList[iCL][(mf.ct.ContourLineList[iCL].Count - 1) / 2].northing, iCL.ToString(), 2);
                        mf.font.DrawText3D(mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 2].easting, mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 2].northing, iCL.ToString(), 2);
                        // look ahead point
                        //mf.font.DrawText3D(mf.guidanceLookPos.easting, mf.guidanceLookPos.northing, "0", 4);
                    }
                }

                GL.LineWidth(3);     // 
                GL.Color3(1.0f, 0.0f, 1.0f);   // violett  driving line
                GL.Begin(PrimitiveType.LineStrip);
                for (int iPoint = 0; iPoint < mf.ct.ContourLineList[howManyPathsAwayiCL].Count; iPoint++)
                {
                    GL.Vertex3(mf.ct.ContourLineList[howManyPathsAwayiCL][iPoint].easting, mf.ct.ContourLineList[howManyPathsAwayiCL][iPoint].northing, 0);
                }
                GL.Disable(EnableCap.LineStipple);
                GL.End();
            }

            GL.LineWidth(2);
            GL.Color3(0.96, 0.2f, 0.2f);   // red   activ AB curve
            GL.Begin(PrimitiveType.LineStrip);
            // draw original AB curve
            for (int h = 0; h < ptCount; h += 10)
            {
                GL.Vertex3(mf.trk.gArr[mf.trk.idx].curvePts[h].easting, mf.trk.gArr[mf.trk.idx].curvePts[h].northing, 0);
            }
            GL.End();
            // draw numbers of linepoints, heading
            GL.LineWidth(2);
            GL.Color3(0.96, 0.2f, 0.2f);   // red   activ AB curve
            GL.Begin(PrimitiveType.LineStrip);
            //ptCountPa = mf.ct.ContourLineList[miCL].Count;
            //for (int h = 0; h < ptCountPa - 1; h += 10)
            //{
            // draw heading of linepoints
            //mf.font.DrawText3D(mf.ct.ContourLineList[howManyPathsAwayiCL][h].easting, mf.ct.ContourLineList[howManyPathsAwayiCL][h].northing, (mf.ct.ContourLineList[howManyPathsAwayiCL][h].heading).ToString("0.####"));//  h.ToString());
            // draw numbers of linepoints
            // mf.font.DrawText3D(mf.ct.ContourLineList[miCL][h].easting, mf.ct.ContourLineList[miCL][h].northing, h.ToString());
            //}
            GL.Disable(EnableCap.LineStipple);
            GL.End();

            // draw AB points from curve
            if ((mf.font.isFontOn) && (mf.trk.gArr[mf.trk.idx].curvePts.Count > 4))
            {
                GL.Color3(0.40f, 0.90f, 0.95f);
                mf.font.DrawText3D(mf.trk.gArr[mf.trk.idx].curvePts[0].easting, mf.trk.gArr[mf.trk.idx].curvePts[0].northing, "&A");
                mf.font.DrawText3D(mf.trk.gArr[mf.trk.idx].curvePts[mf.trk.gArr[mf.trk.idx].curvePts.Count - 1].easting, mf.trk.gArr[mf.trk.idx].curvePts[mf.trk.gArr[mf.trk.idx].curvePts.Count - 1].northing, "&B");
            }
            GL.Disable(EnableCap.LineStipple);
            GL.End();

        }

        public void DrawToolCurve()
        {
            int Minline = mf.GuidanceLine9, Maxline = mf.GuidanceLine9;
            int curvesCount = mf.tooltrk.gToolArr.Count;
            int ptCount = mf.tooltrk.gToolArr[mf.tooltrk.SelectedLineNumber].curve_sowing_Pts.Count;

            if (mf.tooltrk.isbtnAddToolTrackPts)
            {
                // draws the violett AB buildline
                GL.End();
                GL.LineWidth(1);
                //GL.Disable(EnableCap.LineStipple);
                //GL.Color3(0.5f, 0.5f, 1.0f);   //blue 
                GL.LineWidth(1);
                GL.Color3(0.96, 0.2f, 0.2f);   // red   activ AB curve
                GL.Begin(PrimitiveType.LineStrip);
                // draw original AB curve
                for (int i = 0; i < curvesCount; i += 10)
                {
                    for (int h = 0; h < mf.tooltrk.gToolArr[i].curve_sowing_Pts.Count; h += 10)
                    {
                        GL.Vertex3(mf.tooltrk.gToolArr[i].curve_sowing_Pts[h].easting, mf.tooltrk.gToolArr[i].curve_sowing_Pts[h].northing, 0);
                        if (ptCount == i)
                        {
                            GL.LineWidth(2);
                            GL.Color3(1.0f, 0.0f, 1.0f);   // violett  driving curve
                            GL.Begin(PrimitiveType.LineStrip);
                            GL.Vertex3(mf.tooltrk.gToolArr[i].curve_sowing_Pts[h].easting, mf.tooltrk.gToolArr[i].curve_sowing_Pts[h].northing, 0);
                            GL.End();
                        }
                    }

                    if (mf.isLineNumberon)    // linenumbers shown
                    {
                        if (mf.tooltrk.gToolArr[i].curve_sowing_Pts.Count > 10)
                        {
                            GL.Color3(0.96, 0.2f, 0.2f);   // red   activ AB curve
                            mf.font.DrawText3D(mf.tooltrk.gToolArr[i].curve_sowing_Pts[1].easting, mf.tooltrk.gToolArr[i].curve_sowing_Pts[1].northing, i.ToString(), 2);
                            mf.font.DrawText3D(mf.tooltrk.gToolArr[i].curve_sowing_Pts[(mf.tooltrk.gToolArr[i].curve_sowing_Pts.Count - 1) / 2].easting, mf.tooltrk.gToolArr[i].curve_sowing_Pts[(mf.tooltrk.gToolArr[i].curve_sowing_Pts.Count - 1) / 2].northing, i.ToString(), 2);
                            mf.font.DrawText3D(mf.tooltrk.gToolArr[i].curve_sowing_Pts[mf.tooltrk.gToolArr[i].curve_sowing_Pts.Count - 2].easting, mf.tooltrk.gToolArr[i].curve_sowing_Pts[mf.tooltrk.gToolArr[i].curve_sowing_Pts.Count - 2].northing, i.ToString(), 2);
                            // Tool look ahead point
                            //mf.font.DrawText3D(mf.ToolguidanceLookPos.easting, mf.ToolguidanceLookPos.northing, "0", 4);
                        }
                    }

                }
                // numbers of linepoints
                //ptCountPa = mf.ct.ContourLineList[miCL].Count;
                //for (int h = 0; h < ptCountPa - 1; h += 10)
                //{
                // mf.font.DrawText3D(mf.ct.ContourLineList[miCL][h].easting, mf.ct.ContourLineList[miCL][h].northing, h.ToString());
                //}
                GL.Disable(EnableCap.LineStipple);
                GL.End();
            }
            if (mf.tooltrk.isbtnToolTrackStop)
            {
                Console.WriteLine(" mf.tooltrk.isbtnToolTrackStop   ");
                return;
            }
            if (mf.tooltrk.isbtnToolAtWork)
            {
                Console.WriteLine(" mf.tooltrk.isbtnToolAtWork  1 ");
                return;
            }

        }

        public void DrawCurveNew()
        {
            if (desList.Count > 0)
            {
                GL.Color3(0.95f, 0.42f, 0.750f);
                GL.Begin(PrimitiveType.LineStrip);
                for (int h = 0; h < desList.Count; h++) GL.Vertex3(desList[h].easting, desList[h].northing, 0);
                GL.End();
            }
        }

        public void DrawCurve()
        {
            double toolOffset = mf.tool.offset * 2;
            double toolWidth = mf.tool.width - mf.tool.overlap;
            double toolWidthn = mf.tool.width - mf.tool.overlap;
            int ToolNumber;

            if ((mf.isContourPattern) && (track.name.Length > 3 && track.name.Substring(0, 4) == "&Pa "))
            {
                DrawContourPatternCurve();
                return;
            }

            if (desList.Count > 0)
            {
                GL.Color3(0.95f, 0.42f, 0.750f);
                GL.Begin(PrimitiveType.LineStrip);
                for (int h = 0; h < desList.Count; h++) GL.Vertex3(desList[h].easting, desList[h].northing, 0);
                GL.End();
            }

            if (mf.trk.idx == -1) return;

            int ptCount = mf.trk.gArr[mf.trk.idx].curvePts.Count;

            if (mf.trk.gArr[mf.trk.idx].mode != (int)TrackMode.waterPivot)
            {
                if (mf.trk.gArr[mf.trk.idx].curvePts == null || mf.trk.gArr[mf.trk.idx].curvePts.Count == 0) return;

                // draw AB curve red original activ AB curve
                GL.LineWidth(4);
                GL.Color3(0.93, 0.2f, 0.2f);   // red original activ AB curve
                GL.Begin(PrimitiveType.Lines);
                for (int h = 0; h < ptCount; h++)
                    GL.Vertex3(mf.trk.gArr[mf.trk.idx].curvePts[h].easting, mf.trk.gArr[mf.trk.idx].curvePts[h].northing, 0);
                GL.End();

                // draw AB points from curve
                GL.LineWidth(lineWidth * 2);
                if (mf.font.isFontOn)
                {

                    GL.Color3(0.40f, 0.90f, 0.95f);
                    mf.font.DrawText3D(mf.trk.gArr[mf.trk.idx].curvePts[0].easting, mf.trk.gArr[mf.trk.idx].curvePts[0].northing, "&A");
                    mf.font.DrawText3D(mf.trk.gArr[mf.trk.idx].curvePts[mf.trk.gArr[mf.trk.idx].curvePts.Count - 1].easting, mf.trk.gArr[mf.trk.idx].curvePts[mf.trk.gArr[mf.trk.idx].curvePts.Count - 1].northing, "&B");
                }
                if ((mf.isLineNumberon) && (mf.camera.camSetDistance > mf.tool.width * -120))
                {
                    double toolWidthline;
                    toolWidthline = toolWidth;

                    int HalfPointCount = (int)(curList.Count - 1) / 2;

                    for (int toolsn = -mf.GuidanceLine9 + 1; toolsn < mf.GuidanceLine9; toolsn++) //curList[h].easting
                    {
                        toolWidthn = toolWidthline * toolsn;
                        ToolNumber = toolsn + (int)howManyPathsAway + 1;
                        if (ToolNumber <= 0) ToolNumber--;
                        if (curList.Count > 0)
                        {
                            mf.font.DrawText3D((Math.Cos(-curList[10].heading) * (toolWidthn + toolOffset)) + curList[10].easting, (Math.Sin(-curList[10].heading) * (toolWidthn + toolOffset)) + curList[10].northing, ToolNumber.ToString(), 2);
                            mf.font.DrawText3D((Math.Cos(-curList[HalfPointCount].heading) * (toolWidthn + toolOffset)) + curList[HalfPointCount].easting, (Math.Sin(-curList[HalfPointCount].heading) * (toolWidthn + toolOffset)) + curList[HalfPointCount].northing, ToolNumber.ToString(), 2);
                            mf.font.DrawText3D((Math.Cos(-curList[curList.Count - 10].heading) * (toolWidthn + toolOffset)) + curList[curList.Count - 10].easting, (Math.Sin(-curList[curList.Count - 10].heading) * (toolWidthn + toolOffset)) + curList[curList.Count - 10].northing, ToolNumber.ToString());
                        }
                    }
                }

                //just draw ref and smoothed line if smoothing window is open
                if (isSmoothWindowOpen)
                {
                    if (smooList == null || smooList.Count == 0) return;

                    GL.LineWidth(mf.ABLine.lineWidth);
                    GL.Color3(0.930f, 0.92f, 0.260f);
                    GL.Begin(PrimitiveType.Lines);
                    for (int h = 0; h < smooList.Count; h++) GL.Vertex3(smooList[h].easting, smooList[h].northing, 0);
                    GL.End();
                }
                else //normal. Smoothing window is not open.
                {
                    if (curList.Count > 0)
                    {
                        GL.LineWidth(mf.ABLine.lineWidth);  // #######################################
                        GL.Color3(0.95f, 0.2f, 0.95f);

                        //ablines and curves are a line - the rest a loop
                        if (mf.trk.gArr[mf.trk.idx].mode <= (int)TrackMode.Curve)
                        {
                            GL.Begin(PrimitiveType.LineStrip);
                        }
                        else
                        {
                            GL.Begin(PrimitiveType.LineLoop);
                        }
                        // violett drivingline
                        for (int h = 0; h < curList.Count; h++) 
                            GL.Vertex3(curList[h].easting, curList[h].northing, 0);  // whole line under AB Drivecurve

                        CurveroundPoints = curList.Count;

                        GL.End();

                        if (!mf.isStanleyUsed && mf.camera.camSetDistance > -200)
                        {
                            //Draw lookahead Point
                            GL.PointSize(4.0f);
                            GL.Begin(PrimitiveType.Lines); //Points
                            GL.Color3(1.0f, 0.95f, 0.195f);
                            GL.Vertex3(goalPointCu.easting, goalPointCu.northing, 0.0);
                            GL.End();
                        }
                        mf.yt.DrawYouTurn();

                        GL.PointSize(2.0f);
                        GL.Color3(1f, 2.7650f, 0.0050f);
                        GL.Begin(PrimitiveType.Points);//Points

                        //####################################################
                        if ((mf.isSideGuideLines && mf.camera.camSetDistance > mf.tool.width * -120) || (mf.yt.isYouTurnBtnOn))
                        {


                            if (mf.isSideGuideLines)
                            {
                                double toolWidthline;

                                toolWidthline = toolWidth;

                                if (isHeadingSameWay)
                                {
                                    //1. right Line
                                    for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * (toolWidth + toolOffset)) + curList[h].easting, (Math.Sin(-curList[h].heading) * (toolWidth + toolOffset)) + curList[h].northing, 0);
                                    for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * (-toolWidth + toolOffset)) + curList[h].easting, (Math.Sin(-curList[h].heading) * (-toolWidth + toolOffset)) + curList[h].northing, 0);

                                    for (int tools = 2; tools < mf.GuidanceLine9; tools++) //curList[h].easting
                                    {
                                        toolWidth = toolWidthline * tools;
                                        for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * toolWidth) + curList[h].easting, (Math.Sin(-curList[h].heading) * toolWidth) + curList[h].northing, 0);
                                        //for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * toolWidth) + curList[h].easting, (Math.Sin(-curList[h].heading) * toolWidth) + curList[h].northing, 0);
                                        for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * (-toolWidth)) + curList[h].easting, (Math.Sin(-curList[h].heading) * (-toolWidth)) + curList[h].northing, 0);
                                        //for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * (-toolWidth)) + curList[h].easting, (Math.Sin(-curList[h].heading) * (-toolWidth)) + curList[h].northing, 0);
                                        if (tools > 1) GL.Color3(1f, 2.7650f, 0.0050f);
                                        if (tools > 3) GL.Color3(1.0f, 0.000f, 0.50f);
                                        if (tools > 5) GL.Color3(0.0f, 2.7650f, 0.0050f);
                                        if (tools > 7) GL.Color3(1.0f, 1.000f, 1.50f);
                                    }
                                }
                                else
                                {
                                    for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * (toolWidth - toolOffset)) + curList[h].easting, (Math.Sin(-curList[h].heading) * (toolWidth - toolOffset)) + curList[h].northing, 0);
                                    for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * (-toolWidth - toolOffset)) + curList[h].easting, (Math.Sin(-curList[h].heading) * (-toolWidth - toolOffset)) + curList[h].northing, 0);

                                    for (int tools = 2; tools < mf.GuidanceLine9; tools++)
                                    {
                                        toolWidth = toolWidthline * tools;
                                        for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * toolWidth) + curList[h].easting, (Math.Sin(-curList[h].heading) * toolWidth) + curList[h].northing, 0);
                                        for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * (-toolWidth)) + curList[h].easting, (Math.Sin(-curList[h].heading) * (-toolWidth)) + curList[h].northing, 0);
                                        if (tools > 1) GL.Color3(1f, 2.7650f, 0.0050f);
                                        if (tools > 3) GL.Color3(1.0f, 0.000f, 0.50f);
                                        if (tools > 5) GL.Color3(0.0f, 2.7650f, 0.0050f);
                                        if (tools > 7) GL.Color3(1.0f, 1.000f, 1.50f);
                                    }
                                }
                            }
                            else
                            {
                                string nam = mf.trk.gArr[mf.trk.idx].name;

                                if (nam.Length > 0 && nam.Substring(0, 1) != "&")
                                {
                                    if (isHeadingSameWay)
                                    {
                                        Uturncurve_draw();
                                        toolWidth *= mf.yt.rowSkipsWidth2;
                                        if (!mf.yt.isYouTurnRightB)
                                        {
                                            for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * toolWidth) + curList[h].easting, (Math.Sin(-curList[h].heading) * toolWidth) + curList[h].northing, 0);
                                        }
                                        else
                                        {
                                            for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * (-toolWidth)) + curList[h].easting, (Math.Sin(-curList[h].heading) * (-toolWidth)) + curList[h].northing, 0);
                                        }
                                        Uturncurve_nodraw();
                                    }
                                    else
                                    {
                                        Uturncurve_draw();
                                        toolWidth *= mf.yt.rowSkipsWidth1;
                                        if (mf.yt.isYouTurnRightA)
                                        {
                                            for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * toolWidth) + curList[h].easting, (Math.Sin(-curList[h].heading) * toolWidth) + curList[h].northing, 0);
                                        }
                                        else
                                        {
                                            for (int h = 0; h < curList.Count - 1; h++) GL.Vertex3((Math.Cos(-curList[h].heading) * (-toolWidth)) + curList[h].easting, (Math.Sin(-curList[h].heading) * (-toolWidth)) + curList[h].northing, 0);
                                        }
                                        Uturncurve_nodraw();
                                    }
                                }
                            }
                            GL.Disable(EnableCap.LineStipple);

                            GL.End();
                            GL.Disable(EnableCap.LineStipple);


                            GL.End();
                            GL.PointSize(1.0f);
                        }
                    }
                }
            }
            else
            {
                if (curList.Count > 0)
                {
                    GL.LineWidth(mf.ABLine.lineWidth);
                    GL.Color3(0.95f, 0.2f, 0.95f);

                    GL.Begin(PrimitiveType.LineStrip);
                    GL.Vertex3(mf.trk.gArr[mf.trk.idx].ptA.easting, mf.trk.gArr[mf.trk.idx].ptA.northing, 0);
                    for (int h = 0; h < curList.Count; h++) GL.Vertex3(curList[h].easting, curList[h].northing, 0);
                    GL.End();

                    if (!mf.isStanleyUsed && mf.camera.camSetDistance > -200)
                    {
                        //Draw lookahead Point
                        GL.PointSize(8.0f);
                        GL.Begin(PrimitiveType.Points);
                        GL.Color3(1.0f, 0.95f, 0.195f);
                        GL.Vertex3(goalPointCu.easting, goalPointCu.northing, 0.0);
                        GL.End();
                    }
                }
            }
            GL.PointSize(1.0f);
        }

        private void Uturncurve_draw()
        {
            GL.End();
            GL.LineWidth(4);
            //GL.Enable(EnableCap.LineStipple);
            //GL.LineStipple(1, 0x9F20);
            GL.Begin(PrimitiveType.Points);
            //GL.Color3(0.00, 0.00, 0.50);  //blue
            GL.Color3(0.10f, 10.0f, 0.900f);
        }

        private void Uturncurve_nodraw()
        {
            GL.End();
            GL.LineWidth(4);
            //GL.Enable(EnableCap.LineStipple);
            GL.LineStipple(1, 0x0F00);
            GL.Begin(PrimitiveType.Points);
        }

        public void BuildTram()
        {
            //if all or bnd only then make outer loop pass
            if (mf.tram.generateMode != 1)
            {
                mf.tram.BuildTramBnd();
            }
            else
            {
                mf.tram.tramBndOuterArr?.Clear();
                mf.tram.tramBndInnerArr?.Clear();
            }

            mf.tram.tramList?.Clear();
            mf.tram.tramArr?.Clear();

            if (mf.tram.generateMode == 2) return;

            bool isBndExist = mf.bnd.bndList.Count != 0;

            int refCount = mf.trk.gArr[mf.trk.idx].curvePts.Count;

            int cntr = 0;
            if (isBndExist)
            {
                if (mf.tram.generateMode == 1)
                    cntr = 0;
                else
                    cntr = 1;
            }

            double widd = 0;

            for (int i = cntr; i <= mf.tram.passes; i++)
            {
                mf.tram.tramArr = new List<vec2>
                {
                    Capacity = 128
                };

                mf.tram.tramList.Add(mf.tram.tramArr);

                widd = (mf.tram.tramWidth * 0.5) - mf.tram.halfWheelTrack;
                widd += (mf.tram.tramWidth * i);

                double distSqAway = widd * widd * 0.999999;

                for (int j = 0; j < refCount; j += 1)
                {
                    vec2 point = new vec2(
                    (Math.Sin(glm.PIBy2 + mf.trk.gArr[mf.trk.idx].curvePts[j].heading) *
                        widd) + mf.trk.gArr[mf.trk.idx].curvePts[j].easting,
                    (Math.Cos(glm.PIBy2 + mf.trk.gArr[mf.trk.idx].curvePts[j].heading) *
                        widd) + mf.trk.gArr[mf.trk.idx].curvePts[j].northing
                        );

                    bool Add = true;
                    for (int t = 0; t < refCount; t++)
                    {
                        //distance check to be not too close to ref line
                        double dist = ((point.easting - mf.trk.gArr[mf.trk.idx].curvePts[t].easting) * (point.easting - mf.trk.gArr[mf.trk.idx].curvePts[t].easting))
                            + ((point.northing - mf.trk.gArr[mf.trk.idx].curvePts[t].northing) * (point.northing - mf.trk.gArr[mf.trk.idx].curvePts[t].northing));
                        if (dist < distSqAway)
                        {
                            Add = false;
                            break;
                        }
                    }
                    if (Add)
                    {
                        //a new point only every 2 meters
                        double dist = mf.tram.tramArr.Count > 0 ? ((point.easting - mf.tram.tramArr[mf.tram.tramArr.Count - 1].easting) * (point.easting - mf.tram.tramArr[mf.tram.tramArr.Count - 1].easting))
                            + ((point.northing - mf.tram.tramArr[mf.tram.tramArr.Count - 1].northing) * (point.northing - mf.tram.tramArr[mf.tram.tramArr.Count - 1].northing)) : 3.0;
                        if (dist > 2)
                        {
                            //if inside the boundary, add
                            if (!isBndExist || mf.bnd.bndList[0].fenceLineEar.IsPointInPolygon(point))
                            {
                                mf.tram.tramArr.Add(point);
                            }
                        }
                    }
                }
            }

            for (int i = cntr; i <= mf.tram.passes; i++)
            {
                mf.tram.tramArr = new List<vec2>
                {
                    Capacity = 128
                };

                mf.tram.tramList.Add(mf.tram.tramArr);

                widd = (mf.tram.tramWidth * 0.5) + mf.tram.halfWheelTrack;
                widd += (mf.tram.tramWidth * i);
                double distSqAway = widd * widd * 0.999999;

                for (int j = 0; j < refCount; j += 1)
                {
                    vec2 point = new vec2(
                    Math.Sin(glm.PIBy2 + mf.trk.gArr[mf.trk.idx].curvePts[j].heading) *
                        widd + mf.trk.gArr[mf.trk.idx].curvePts[j].easting,
                    Math.Cos(glm.PIBy2 + mf.trk.gArr[mf.trk.idx].curvePts[j].heading) *
                        widd + mf.trk.gArr[mf.trk.idx].curvePts[j].northing
                        );

                    bool Add = true;
                    for (int t = 0; t < refCount; t++)
                    {
                        //distance check to be not too close to ref line
                        double dist = ((point.easting - mf.trk.gArr[mf.trk.idx].curvePts[t].easting) * (point.easting - mf.trk.gArr[mf.trk.idx].curvePts[t].easting))
                            + ((point.northing - mf.trk.gArr[mf.trk.idx].curvePts[t].northing) * (point.northing - mf.trk.gArr[mf.trk.idx].curvePts[t].northing));
                        if (dist < distSqAway)
                        {
                            Add = false;
                            break;
                        }
                    }
                    if (Add)
                    {
                        //a new point only every 2 meters
                        double dist = mf.tram.tramArr.Count > 0 ? ((point.easting - mf.tram.tramArr[mf.tram.tramArr.Count - 1].easting) * (point.easting - mf.tram.tramArr[mf.tram.tramArr.Count - 1].easting))
                            + ((point.northing - mf.tram.tramArr[mf.tram.tramArr.Count - 1].northing) * (point.northing - mf.tram.tramArr[mf.tram.tramArr.Count - 1].northing)) : 3.0;
                        if (dist > 2)
                        {
                            //if inside the boundary, add
                            if (!isBndExist || mf.bnd.bndList[0].fenceLineEar.IsPointInPolygon(point))
                            {
                                mf.tram.tramArr.Add(point);
                            }
                        }
                    }
                }
            }
        }

        //for calculating for display the averaged new line
        public void SmoothAB(int smPts)
        {
            //count the reference list of original curve
            int cnt = mf.trk.gArr[mf.trk.idx].curvePts.Count;

            //just go back if not very long
            if (cnt < 100) return;

            //the temp array
            vec3[] arr = new vec3[cnt];

            //read the points before and after the setpoint
            for (int s = 0; s < smPts / 2; s++)
            {
                arr[s].easting = mf.trk.gArr[mf.trk.idx].curvePts[s].easting;
                arr[s].northing = mf.trk.gArr[mf.trk.idx].curvePts[s].northing;
                arr[s].heading = mf.trk.gArr[mf.trk.idx].curvePts[s].heading;
            }

            for (int s = cnt - (smPts / 2); s < cnt; s++)
            {
                arr[s].easting = mf.trk.gArr[mf.trk.idx].curvePts[s].easting;
                arr[s].northing = mf.trk.gArr[mf.trk.idx].curvePts[s].northing;
                arr[s].heading = mf.trk.gArr[mf.trk.idx].curvePts[s].heading;
            }

            //average them - center weighted average
            for (int i = smPts / 2; i < cnt - (smPts / 2); i++)
            {
                for (int j = -smPts / 2; j < smPts / 2; j++)
                {
                    arr[i].easting += mf.trk.gArr[mf.trk.idx].curvePts[j + i].easting;
                    arr[i].northing += mf.trk.gArr[mf.trk.idx].curvePts[j + i].northing;
                }
                arr[i].easting /= smPts;
                arr[i].northing /= smPts;
                arr[i].heading = mf.trk.gArr[mf.trk.idx].curvePts[i].heading;
            }

            //make a list to draw
            smooList?.Clear();

            if (arr == null || cnt < 1) return;
            if (smooList == null) return;

            for (int i = 0; i < cnt; i++)
            {
                smooList.Add(arr[i]);
            }
        }

        public void CalculateHeadings(ref List<vec3> xList)
        {
            //to calc heading based on next and previous points to give an average heading.
            int cnt = xList.Count;
            if (cnt > 3)
            {
                vec3[] arr = new vec3[cnt];
                cnt--;
                xList.CopyTo(arr);
                xList.Clear();

                vec3 pt3 = arr[0];
                pt3.heading = Math.Atan2(arr[1].easting - arr[0].easting, arr[1].northing - arr[0].northing);
                if (pt3.heading < 0) pt3.heading += glm.twoPI;
                xList.Add(pt3);

                //middle points
                for (int i = 1; i < cnt; i++)
                {
                    pt3 = arr[i];
                    pt3.heading = Math.Atan2(arr[i + 1].easting - arr[i - 1].easting, arr[i + 1].northing - arr[i - 1].northing);
                    if (pt3.heading < 0) pt3.heading += glm.twoPI;
                    xList.Add(pt3);
                }

                pt3 = arr[arr.Length - 1];
                pt3.heading = Math.Atan2(arr[arr.Length - 1].easting - arr[arr.Length - 2].easting,
                    arr[arr.Length - 1].northing - arr[arr.Length - 2].northing);
                if (pt3.heading < 0) pt3.heading += glm.twoPI;
                xList.Add(pt3);
            }
        }

        public void MakePointMinimumSpacing(ref List<vec3> xList, double minDistance)
        {
            int cnt = xList.Count;
            if (cnt > 3)
            {
                //make sure point distance isn't too big
                for (int i = 0; i < cnt - 1; i++)
                {
                    int j = i + 1;
                    //if (j == cnt) j = 0;
                    double distance = glm.Distance(xList[i], xList[j]);
                    if (distance > minDistance)
                    {
                        vec3 pointB = new vec3((xList[i].easting + xList[j].easting) / 2.0,
                            (xList[i].northing + xList[j].northing) / 2.0,
                            xList[i].heading);

                        xList.Insert(j, pointB);
                        cnt = xList.Count;
                        i = -1;
                    }
                }
            }
        }


        //turning the visual line into the real reference line to use
        public void SaveSmoothList()
        {
            //oops no smooth list generated
            if (smooList == null) return;
            int cnt = smooList.Count;
            if (cnt == 0) return;

            //eek
            mf.trk.gArr[mf.trk.idx].curvePts?.Clear();

            //copy to an array to calculate all the new headings
            vec3[] arr = new vec3[cnt];
            smooList.CopyTo(arr);

            //calculate new headings on smoothed line
            for (int i = 1; i < cnt - 1; i++)
            {
                arr[i].heading = Math.Atan2(arr[i + 1].easting - arr[i].easting, arr[i + 1].northing - arr[i].northing);
                if (arr[i].heading < 0) arr[i].heading += glm.twoPI;
                mf.trk.gArr[mf.trk.idx].curvePts.Add(arr[i]);
            }
        }

        public bool PointOnLine(vec3 pt1, vec3 pt2, vec3 pt)
        {
            vec2 r = new vec2(0, 0);
            if (pt1.northing == pt2.northing && pt1.easting == pt2.easting) { pt1.northing -= 0.00001; }

            double U = ((pt.northing - pt1.northing) * (pt2.northing - pt1.northing)) + ((pt.easting - pt1.easting) * (pt2.easting - pt1.easting));

            double Udenom = Math.Pow(pt2.northing - pt1.northing, 2) + Math.Pow(pt2.easting - pt1.easting, 2);

            U /= Udenom;

            r.northing = pt1.northing + (U * (pt2.northing - pt1.northing));
            r.easting = pt1.easting + (U * (pt2.easting - pt1.easting));

            double minx, maxx, miny, maxy;

            minx = Math.Min(pt1.northing, pt2.northing);
            maxx = Math.Max(pt1.northing, pt2.northing);

            miny = Math.Min(pt1.easting, pt2.easting);
            maxy = Math.Max(pt1.easting, pt2.easting);
            return _ = r.northing >= minx && r.northing <= maxx && (r.easting >= miny && r.easting <= maxy);
        }

        //add extensons
        public void AddFirstLastPoints(ref List<vec3> xList)
        {
            int ptCnt = xList.Count - 1;
            vec3 start = new vec3(xList[0]);

            if (mf.bnd.bndList.Count > 0)
            {
                for (int i = 1; i < 100; i++)
                {
                    vec3 pt = new vec3(xList[ptCnt]);
                    pt.easting += (Math.Sin(pt.heading) * i);
                    pt.northing += (Math.Cos(pt.heading) * i);
                    xList.Add(pt);
                }

                //and the beginning
                start = new vec3(xList[0]);

                for (int i = 1; i < 100; i++)
                {
                    vec3 pt = new vec3(start);
                    pt.easting -= (Math.Sin(pt.heading) * i);
                    pt.northing -= (Math.Cos(pt.heading) * i);
                    xList.Insert(0, pt);
                }

            }
            else
            {
                for (int i = 1; i < 300; i++)
                {
                    vec3 pt = new vec3(xList[ptCnt]);
                    pt.easting += (Math.Sin(pt.heading) * i);
                    pt.northing += (Math.Cos(pt.heading) * i);
                    xList.Add(pt);
                }

                //and the beginning
                start = new vec3(xList[0]);

                for (int i = 1; i < 300; i++)
                {
                    vec3 pt = new vec3(start);
                    pt.easting -= (Math.Sin(pt.heading) * i);
                    pt.northing -= (Math.Cos(pt.heading) * i);
                    xList.Insert(0, pt);
                }
            }
        }

        public void ResetCurveLine()
        {
            curList?.Clear();
            mf.trk.idx = -1;
        }
        public class CCurveLines
        {
            public List<vec3> curvePts = new List<vec3>();
            public double aveHeading = 3;
            public string Name = "aa";
        }
    }
}