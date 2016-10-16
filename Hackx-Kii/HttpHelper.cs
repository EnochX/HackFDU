using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Newtonsoft.Json;
using System.Threading.Tasks;

namespace HttpModule
{
    class HttpHelper
    {

        private static string WebServerHttpAddr = "http://139.219.232.75:8080/FYBody_Server/";

        private static HttpHelper mHttpHelper;

        private HttpHelper() {

        }

        public static HttpHelper getInstance()
        {
            if (mHttpHelper == null)
            {
                mHttpHelper = new HttpHelper();
                return mHttpHelper;
            }
            return mHttpHelper;
        }

        public bool record(string uname, string aname)
        {
            string url = WebServerHttpAddr + "Record.action?uname=" + uname + "&aname=" + aname;
            string retval = HttpRequest.Get(url);

            if (retval != null)
            {
                JsonReader reader = new JsonTextReader(new StringReader(retval));
                while (reader.Read())
                {
                    if (reader.Path.ToString().Equals("result.success"))
                    {
                        reader.Read();
                        string success = (string)reader.Value;
                        if (success.Equals("true"))
                        {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        public string getRecords(string uname, int year, int month)
        {
            string url = WebServerHttpAddr + "GetRecords.action?uname=" + uname + "&year=" + year + "&month=" + month;
            string retval = HttpRequest.Get(url);

            if (retval != null)
            {
                JsonReader reader = new JsonTextReader(new StringReader(retval));
                while (reader.Read())
                {
                    if (reader.Path.ToString().Equals("result.success"))
                    {
                        reader.Read();
                        string success = (string) reader.Value;
                        if (success.Equals("true"))
                        {
                            reader.Read();
                            if (reader.Path.ToString().Equals("result.response"))
                            {
                                reader.Read();
                                return (string)reader.Value;
                            }
                        }                   

                    }
                }
            }
            return "Error";
        }

    }
}
