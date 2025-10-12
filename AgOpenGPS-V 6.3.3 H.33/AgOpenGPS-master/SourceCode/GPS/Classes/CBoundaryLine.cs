using System.Collections.Generic;

namespace AgOpenGPS.Classes
{
    public class CBoundaryLine
    {
        private readonly FormGPS mf;

        public List<CBoundaryPath> tracksArrbndl = new List<CBoundaryPath>();
        public List<CBoundaryCorner> tracksCorner = new List<CBoundaryCorner>();

        public int idxbndl;

        public List<vec3> desListbndl = new List<vec3>();

        public CBoundaryLine(FormGPS _f)
        {
            //constructor
            mf = _f;
        }
        //for calculating for display the averaged new line

    }

    public class CBoundaryPath
    {
        public List<vec3> trackPtsbndl = new List<vec3>();
        public string name = "";
        public double moveDistance = 0;
        public int mode = 0;
        public int a_point = 0;
    }
    public class CBoundaryCorner
    {
        public List<vec3> PtsCorner = new List<vec3>();
        public string name = "";
        public double number = 0;
        public int a_point = 0;
        public double LineA = 0;
        public double LineB = 0;
        public bool exist = false;
    }

}
