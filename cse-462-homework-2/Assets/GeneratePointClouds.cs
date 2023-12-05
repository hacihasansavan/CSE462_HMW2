using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GeneratePointClouds : MonoBehaviour
{
    [SerializeField] string fileName;
    [SerializeField] Material material;

    void Awake()
    {
        string path = System.IO.Path.Combine(Application.dataPath, "Resources", fileName + ".txt");

        try
        {
            string[] lines = System.IO.File.ReadAllLines(path);

            if (lines.Length < 1)
            {
                Debug.LogError("Invalid file format. The file should contain at least one line.");
                return;
            }

            if (!int.TryParse(lines[0], out int n))
            {
                Debug.LogError("Invalid value for the number of points.");
                return;
            }

            int sumX = 0;
            int sumY = 0;
            int sumZ = 0;

            for (int i = 1; i <= n && i < lines.Length; i++)
            {
                string[] line = lines[i].Split(' ');

                if (line.Length < 3)
                {
                    Debug.LogError("Invalid line format at line " + i);
                    continue;
                }

                if (TryParseFloat(line[0], out float x) &&
                    TryParseFloat(line[1], out float y) &&
                    TryParseFloat(line[2], out float z))
                {
                    sumX += (int)x;
                    sumY += (int)y;
                    sumZ += (int)z;

                    GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    sphere.transform.position = new Vector3(x, y, z);
                    sphere.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
                    sphere.GetComponent<Renderer>().material = material;
                    sphere.name = "P " + i;
                    sphere.transform.parent = transform;
                }
                else
                {
                    Debug.LogError("Failed to parse coordinates at line " + i);
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError("Error reading file: " + e.Message);
        }
    }

    bool TryParseFloat(string s, out float result)
    {
        return float.TryParse(s, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out result);
    }
}
