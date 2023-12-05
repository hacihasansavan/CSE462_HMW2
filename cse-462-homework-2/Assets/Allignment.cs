using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Accord.Math;
using Accord.Math.Decompositions;
using Accord.Imaging;
using Accord.Math.Geometry;
using UnityEngine.UI;
using TMPro;


public class Allignment : MonoBehaviour
{
    public GameObject pc1;
    public GameObject pc2;
    private float threshold = 0.3f;
    private int iteration = 10000;
    private bool toEnd = true;


    private List<Accord.Math.Vector3> points_1;
    private List<Accord.Math.Vector3> points_2;
    //private List<Accord.Math.Vector3> points_3;

    void Start()
    {
        points_1 = GetPointListFromTransform(pc1.transform);
        points_2 = GetPointListFromTransform(pc2.transform);
        //points_3 = GetPointListFromTransform(pc2.transform);

    }

    List<Accord.Math.Vector3> GetPointListFromTransform(Transform pointCloudTransform)
    {
        List<Accord.Math.Vector3> points = new List<Accord.Math.Vector3>();

        for (int i = 0; i < pointCloudTransform.childCount; i++)
        {
            points.Add(new Accord.Math.Vector3(
                pointCloudTransform.GetChild(i).position.x,
                pointCloudTransform.GetChild(i).position.y,
                pointCloudTransform.GetChild(i).position.z
            ));
        }

        return points;
    }


    public void StartProcess()
    {
        Debug.Log("Processing...");
        // Get the best transformation
        Accord.Math.Vector3 translation;
        Accord.Math.Matrix3x3 rotation;
        float scale;
        BestTransform(points_1, points_2, out translation, out rotation);

        // Apply the transformation to the point cloud 2
        MakeTransformation(pc2, rotation, translation);
        //startButton.GetComponent<TMPro.TextMeshPro>().text = "End";
        Debug.Log("END");


    }

    void MakeTransformation(GameObject pointCloud, Accord.Math.Matrix3x3 rotation, Accord.Math.Vector3 translation)
    {
        for (int i = 0; i < pointCloud.transform.childCount; i++)
        {
            Accord.Math.Vector3 point = new Accord.Math.Vector3(pointCloud.transform.GetChild(i).position.x, pointCloud.transform.GetChild(i).position.y, pointCloud.transform.GetChild(i).position.z);
            point = rotation * point;
            point = point + translation;

            pointCloud.transform.GetChild(i).gameObject.AddComponent<LineRenderer>();
            LineRenderer lr = pointCloud.transform.GetChild(i).GetComponent<LineRenderer>();
            lr.material = new Material(Shader.Find("Sprites/Default"));
            lr.startColor = Color.black;
            lr.endColor = Color.black;
            lr.startWidth = 0.05f;
            lr.endWidth = 0.05f;
            lr.SetPosition(0, pointCloud.transform.GetChild(i).position);
            lr.SetPosition(1, new UnityEngine.Vector3(point.X, point.Y, point.Z));

            pointCloud.transform.GetChild(i).position = new UnityEngine.Vector3(point.X, point.Y, point.Z);

        }

    }


    int NumOfOverlappings(List<Accord.Math.Vector3> points_1, List<Accord.Math.Vector3> points_2, float threshold)
    {
        int overlappingPoints = 0;
        int[] overlappingPoints_1 = new int[points_1.Count];

        for (int i = 0; i < points_1.Count; i++)
        {
            for (int j = 0; j < points_2.Count; j++)
            {
                if (Distance(points_1[i], points_2[j]) < threshold && overlappingPoints_1[i] == 0)
                {
                    overlappingPoints++;
                    overlappingPoints_1[i] = 1;
                    break;
                }
            }
        }

        return overlappingPoints;
    }

    Accord.Math.Matrix3x3 ComputeH(Accord.Math.Matrix3x3 matrix1, Accord.Math.Matrix3x3 matrix2)
    {
        return matrix1 * matrix2.Transpose();
    }

    Accord.Math.Matrix3x3 ComputeRotationMatrix(Accord.Math.Matrix3x3 H)
    {
        H.SVD(out Accord.Math.Matrix3x3 U, out Accord.Math.Vector3 S, out Accord.Math.Matrix3x3 V);
        Accord.Math.Matrix3x3 rotationMatrix = V * U.Transpose();

        if (Determinant(rotationMatrix) < 0)
        {
            rotationMatrix.SVD(out Accord.Math.Matrix3x3 U_temp, out Accord.Math.Vector3 S_temp, out Accord.Math.Matrix3x3 V_temp);
            V_temp.V20 = -V_temp.V00;
            V_temp.V21 = -V_temp.V01;
            V_temp.V22 = -V_temp.V02;
            rotationMatrix = V_temp * U_temp.Transpose();
        }

        return rotationMatrix;
    }

    float ComputeScale(Accord.Math.Matrix3x3 H)
    {
        H.SVD(out Accord.Math.Matrix3x3 U, out Accord.Math.Vector3 S, out Accord.Math.Matrix3x3 V);
        return (S.X + S.Y + S.Z) / 3f;
    }



    Accord.Math.Vector3 ComputeTranslation(Accord.Math.Vector3 centroid1, Accord.Math.Vector3 centroid2, Accord.Math.Matrix3x3 rotationMatrix)
    {
        return centroid2 - rotationMatrix * centroid1;
    }


    void GetTransform(Accord.Math.Vector3 point_1_1, Accord.Math.Vector3 point_1_2, Accord.Math.Vector3 point_1_3, Accord.Math.Vector3 point_2_1, Accord.Math.Vector3 point_2_2, Accord.Math.Vector3 point_2_3, out Accord.Math.Vector3 translation, out Accord.Math.Matrix3x3 rotation, out float scale)
    {

        Accord.Math.Vector3 centroid_1 = new Accord.Math.Vector3();
        centroid_1.X = (point_1_1.X + point_1_2.X + point_1_3.X) / 3;
        centroid_1.Y = (point_1_1.Y + point_1_2.Y + point_1_3.Y) / 3;
        centroid_1.Z = (point_1_1.Z + point_1_2.Z + point_1_3.Z) / 3;

        Accord.Math.Vector3 centroid_2 = new Accord.Math.Vector3();
        centroid_2.X = (point_2_1.X + point_2_2.X + point_2_3.X) / 3;
        centroid_2.Y = (point_2_1.Y + point_2_2.Y + point_2_3.Y) / 3;
        centroid_2.Z = (point_2_1.Z + point_2_2.Z + point_2_3.Z) / 3;

        Accord.Math.Vector3 point_1_1_relative = new Accord.Math.Vector3();
        point_1_1_relative.X = point_1_1.X - centroid_1.X;
        point_1_1_relative.Y = point_1_1.Y - centroid_1.Y;
        point_1_1_relative.Z = point_1_1.Z - centroid_1.Z;

        Accord.Math.Vector3 point_1_2_relative = new Accord.Math.Vector3();
        point_1_2_relative.X = point_1_2.X - centroid_1.X;
        point_1_2_relative.Y = point_1_2.Y - centroid_1.Y;
        point_1_2_relative.Z = point_1_2.Z - centroid_1.Z;

        Accord.Math.Vector3 point_1_3_relative = new Accord.Math.Vector3();
        point_1_3_relative.X = point_1_3.X - centroid_1.X;
        point_1_3_relative.Y = point_1_3.Y - centroid_1.Y;
        point_1_3_relative.Z = point_1_3.Z - centroid_1.Z;

        Accord.Math.Vector3 point_2_1_relative = new Accord.Math.Vector3();
        point_2_1_relative.X = point_2_1.X - centroid_2.X;
        point_2_1_relative.Y = point_2_1.Y - centroid_2.Y;
        point_2_1_relative.Z = point_2_1.Z - centroid_2.Z;

        Accord.Math.Vector3 point_2_2_relative = new Accord.Math.Vector3();
        point_2_2_relative.X = point_2_2.X - centroid_2.X;
        point_2_2_relative.Y = point_2_2.Y - centroid_2.Y;
        point_2_2_relative.Z = point_2_2.Z - centroid_2.Z;

        Accord.Math.Vector3 point_2_3_relative = new Accord.Math.Vector3();
        point_2_3_relative.X = point_2_3.X - centroid_2.X;
        point_2_3_relative.Y = point_2_3.Y - centroid_2.Y;
        point_2_3_relative.Z = point_2_3.Z - centroid_2.Z;

        Accord.Math.Matrix3x3 matrix_1 = new Accord.Math.Matrix3x3();
        matrix_1.V00 = point_1_1_relative.X;
        matrix_1.V01 = point_1_1_relative.Y;
        matrix_1.V02 = point_1_1_relative.Z;
        matrix_1.V10 = point_1_2_relative.X;
        matrix_1.V11 = point_1_2_relative.Y;
        matrix_1.V12 = point_1_2_relative.Z;
        matrix_1.V20 = point_1_3_relative.X;
        matrix_1.V21 = point_1_3_relative.Y;
        matrix_1.V22 = point_1_3_relative.Z;

        Accord.Math.Matrix3x3 matrix_2 = new Accord.Math.Matrix3x3();
        matrix_2.V00 = point_2_1_relative.X;
        matrix_2.V01 = point_2_1_relative.Y;
        matrix_2.V02 = point_2_1_relative.Z;
        matrix_2.V10 = point_2_2_relative.X;
        matrix_2.V11 = point_2_2_relative.Y;
        matrix_2.V12 = point_2_2_relative.Z;
        matrix_2.V20 = point_2_3_relative.X;
        matrix_2.V21 = point_2_3_relative.Y;
        matrix_2.V22 = point_2_3_relative.Z;

        Accord.Math.Matrix3x3 H = new Accord.Math.Matrix3x3();
        H = matrix_1 * matrix_2.Transpose();

        H.SVD(out Accord.Math.Matrix3x3 U, out Accord.Math.Vector3 S, out Accord.Math.Matrix3x3 V);

        Accord.Math.Matrix3x3 R = new Accord.Math.Matrix3x3();
        R = V * U.Transpose();

        if (Determinant(R) < 0)
        {
            R.SVD(out Accord.Math.Matrix3x3 U_temp, out Accord.Math.Vector3 S_temp, out Accord.Math.Matrix3x3 V_temp);
            V_temp.V20 = -V_temp.V00;
            V_temp.V21 = -V_temp.V01;
            V_temp.V22 = -V_temp.V02;
            R = V_temp * U_temp.Transpose();
        }

        float scale_temp = (S.X + S.Y + S.Z) / 3f;
        translation = centroid_2 - R * centroid_1;
        rotation = R;
        scale = scale_temp;
    }
    void BestTransform(List<Accord.Math.Vector3> pointVec1, List<Accord.Math.Vector3> pointVec2,
        out Accord.Math.Vector3 translation, out Accord.Math.Matrix3x3 rotation)
    {

        int bestInliers = 0;
        Accord.Math.Vector3 bestTranslation = new Accord.Math.Vector3();
        bestTranslation.X = 0;
        bestTranslation.Y = 0;
        bestTranslation.Z = 0;

        Accord.Math.Matrix3x3 bestRotation = new Accord.Math.Matrix3x3();
        bestRotation.V00 = 1;
        bestRotation.V01 = 0;
        bestRotation.V02 = 0;
        bestRotation.V10 = 0;
        bestRotation.V11 = 1;
        bestRotation.V12 = 0;
        bestRotation.V20 = 0;
        bestRotation.V21 = 0;
        bestRotation.V22 = 1;

        float bestScale = 1;

        for (int i = 0; i < iteration || toEnd; i++)
        {


            int index_1_1 = Random.Range(0, pointVec1.Count);
            int index_1_2 = Random.Range(0, pointVec1.Count);
            int index_1_3 = Random.Range(0, pointVec1.Count);

            int index_2_1 = Random.Range(0, pointVec2.Count);
            int index_2_2 = Random.Range(0, pointVec2.Count);
            int index_2_3 = Random.Range(0, pointVec2.Count);

            Accord.Math.Vector3 point_1_1 = pointVec1[index_1_1];
            Accord.Math.Vector3 point_1_2 = pointVec1[index_1_2];
            Accord.Math.Vector3 point_1_3 = pointVec1[index_1_3];

            Accord.Math.Vector3 point_2_1 = pointVec2[index_2_1];
            Accord.Math.Vector3 point_2_2 = pointVec2[index_2_2];
            Accord.Math.Vector3 point_2_3 = pointVec2[index_2_3];

            Accord.Math.Vector3 translation_temp;
            Accord.Math.Matrix3x3 rotation_temp;
            float scale_temp;

            GetTransform(point_1_1, point_1_2, point_1_3, point_2_1, point_2_2, point_2_3, out translation_temp, out rotation_temp, out scale_temp);

            Accord.Math.Matrix3x3 scaleMatrix = new Accord.Math.Matrix3x3();
            scaleMatrix.V00 = scale_temp;
            scaleMatrix.V01 = 0;
            scaleMatrix.V02 = 0;
            scaleMatrix.V10 = 0;
            scaleMatrix.V11 = scale_temp;
            scaleMatrix.V12 = 0;
            scaleMatrix.V20 = 0;
            scaleMatrix.V21 = 0;
            scaleMatrix.V22 = scale_temp;

            List<Accord.Math.Vector3> points_2_temp = new List<Accord.Math.Vector3>();
            for (int j = 0; j < pointVec2.Count; j++)
            {
                Accord.Math.Vector3 point_temp = rotation_temp * pointVec2[j] + translation_temp;
                points_2_temp.Add(point_temp);
            }

            int inliers = 0;
            for (int j = 0; j < pointVec1.Count; j++)
            {
                inliers = NumOfOverlappings(pointVec1, points_2_temp, threshold);
            }

            // Update the best transformation
            if (inliers > bestInliers)
            {
                bestInliers = inliers;
                bestTranslation = translation_temp;
                bestRotation = rotation_temp;
                bestScale = scale_temp;
            }

            if (bestInliers > pointVec1.Count / 2) break;
        }

        translation = bestTranslation;
        rotation = bestRotation;
    }


    public float Determinant(Accord.Math.Matrix3x3 m)
    {
        return m.V00 * (m.V11 * m.V22 - m.V12 * m.V21) - m.V01 * (m.V10 * m.V22 - m.V12 * m.V20) + m.V02 * (m.V10 * m.V21 - m.V11 * m.V20);
    }

    public float Distance(Accord.Math.Vector3 point_1, Accord.Math.Vector3 point_2)
    {
        return (float)Mathf.Sqrt(Mathf.Pow(point_1.X - point_2.X, 2) + Mathf.Pow(point_1.Y - point_2.Y, 2) + Mathf.Pow(point_1.Z - point_2.Z, 2));
    }


}
