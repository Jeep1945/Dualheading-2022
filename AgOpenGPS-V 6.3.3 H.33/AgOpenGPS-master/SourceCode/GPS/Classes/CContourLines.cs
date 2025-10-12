using System.Collections.Generic;

namespace AgOpenGPS
{
    public class CContourLines
    {
        private readonly FormGPS mf;

        public List<CContourLinesPath> tracksArrCont = new List<CContourLinesPath>();

        public int idxCont;

        public List<vec3> desListCont = new List<vec3>();

        public CContourLines(FormGPS _f)
        {
            //constructor
            mf = _f;
        }
    }
    //for calculating for display the averaged new line



    public class CContourLinesPath
    {
        public List<vec3> trackPtsCont = new List<vec3>();
        public string name = "";
        public double moveDistance = 0;
        public int mode = 0;
        public int a_point = 0;
    }

}
