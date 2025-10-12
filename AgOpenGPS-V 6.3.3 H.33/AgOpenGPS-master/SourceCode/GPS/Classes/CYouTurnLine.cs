using System.Collections.Generic;

namespace AgOpenGPS
{
    public class CYouTurnLine
    {
        private readonly FormGPS mf;

        public List<CYouTurnPath> tracksArryt = new List<CYouTurnPath>();

        public int idxyt;

        public List<vec3> desListyt = new List<vec3>();

        public CYouTurnLine(FormGPS _f)
        {
            //constructor
            mf = _f;
        }

        //for calculating for display the averaged new line

    }

    public class CYouTurnPath
    {
        public List<vec3> trackPtsyt = new List<vec3>();
        public string name = "";
        public double moveDistance = 0;
        public int mode = 0;
        public int a_point = 0;
    }

}
