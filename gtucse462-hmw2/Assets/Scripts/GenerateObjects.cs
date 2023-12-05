using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class GenerateObjects : MonoBehaviour
{
    [SerializeField] string fileName;
    [SerializeField] Material material;

    void Start()
    {
        Debug.Log("FileName : "+ fileName);
        string path = Application.dataPath + "/Resources/" + fileName + ".txt";
        string[] lines = System.IO.File.ReadAllLines(path);

        int n = int.Parse(lines[0]);
        int sumX = 0;
        int sumY = 0;
        int sumZ = 0;

        for (int i = 1; i <= n; i++)
        {
            string[] line = lines[i].Split(' ');
            float x = ToFloat(line[0]);
            float y = ToFloat(line[1]);
            float z = ToFloat(line[2]);

            sumX += (int)x;
            sumY += (int)y;
            sumZ += (int)z;

            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
            sphere.transform.position = new Vector3(x, y, z);
            // sphere.transform.localScale = new Vector3(0.7f, 0.7f, 0.7f);
            sphere.GetComponent<Renderer>().material = material;
            sphere.name = "P " + i;
            sphere.transform.parent = transform;
        }
    }

    float ToFloat(string s)
    {
        return float.Parse(s, System.Globalization.CultureInfo.InvariantCulture);
    }
}
