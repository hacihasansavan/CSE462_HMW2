using System.Collections.Generic;
using UnityEngine;
using Accord.Math;
using Accord.Math.Decompositions;
using Accord.Imaging;
using Accord.Math.Geometry;

public class AlignmentMAnager : MonoBehaviour
{
    public GameObject pointCloud1;
    public GameObject pointCloud2;
    public float threshold = 0.3f;
    public bool transformPointCloud = true;
    public bool drawTransformLine = true;
    public int iteration = 10000;
    public bool inf = true;
    public bool run = false;

    private bool isAligned = false;
    private List<Vector3> points1;
    private List<Vector3> points2;

    void Start()
    {
        points1 = ExtractPointsFromGameObject(pointCloud1);
        points2 = ExtractPointsFromGameObject(pointCloud2);
    }

    void Update()
    {
        if (!isAligned && run)
        {
            AlignPointClouds();
        }
    }

    List<Vector3> ExtractPointsFromGameObject(GameObject pointCloud)
    {
        List<Vector3> points = new List<Vector3>();

        for (int i = 0; i < pointCloud.transform.childCount; i++)
        {
            Transform childTransform = pointCloud.transform.GetChild(i);
            points.Add(childTransform.position);
        }

        return points;
    }

    void AlignPointClouds()
    {
        Accord.Math.Vector3 translation;
        Accord.Math.Matrix3x3 rotation;
        float scale;
        GetBestTransformation(points1, points2, out translation, out rotation, out scale);

        ApplyTransformationToPointCloud(points2, translation, rotation, scale);

        isAligned = true;
        Debug.Log("PointClouds Aligned");
    }

    void GetBestTransformation(List<Vector3> points1, List<Vector3> points2, out Accord.Math.Vector3 translation, out Accord.Math.Matrix3x3 rotation, out float scale)
    {
        int bestInliers = 0;
        translation = new Accord.Math.Vector3(0, 0, 0);
        rotation = Matrix4x4.identity;
        scale = 1;

        for (int i = 0; i < iteration || inf; i++)
        {
            Debug.Log("Iteration");

            int[] randomIndices1 = GetRandomIndices(points1.Count, 3);
            int[] randomIndices2 = GetRandomIndices(points2.Count, 3);

            List<Vector3> subsetPoints1 = ExtractSubset(points1, randomIndices1);
            List<Vector3> subsetPoints2 = ExtractSubset(points2, randomIndices2);

            Accord.Math.Vector3 tempTranslation;
            Accord.Math.Matrix3x3 tempRotation;
            float tempScale;

            GetTransformation(subsetPoints1, subsetPoints2, out tempTranslation, out tempRotation, out tempScale);

            List<Vector3> transformedPoints2 = ApplyTransformation(subsetPoints2, tempTranslation, tempRotation, tempScale);

            int inliers = CountOverlappingPoints(points1, transformedPoints2, threshold);

            if (inliers > bestInliers)
            {
                bestInliers = inliers;
                translation = tempTranslation;
                rotation = tempRotation;
                scale = tempScale;
            }

            if (bestInliers > points1.Count / 2)
                break;
        }

        Debug.Log("Best inliers: " + bestInliers);
    }

    int[] GetRandomIndices(int count, int subsetSize)
    {
        int[] indices = new int[subsetSize];
        for (int i = 0; i < subsetSize; i++)
        {
            indices[i] = Random.Range(0, count);
        }
        return indices;
    }

    List<Vector3> ExtractSubset(List<Vector3> points, int[] indices)
    {
        List<Vector3> subset = new List<Vector3>();
        foreach (int index in indices)
        {
            subset.Add(points[index]);
        }
        return subset;
    }

    void GetTransformation(List<Vector3> points1, List<Vector3> points2, out Accord.Math.Vector3 translation, out Accord.Math.Matrix3x3 rotation, out float scale)
    {
        // Calculate centroids of both point clouds
        Accord.Math.Vector3 centroid1 = CalculateCentroid(points1);
        Accord.Math.Vector3 centroid2 = CalculateCentroid(points2);

        // Calculate relative points with respect to centroids
        List<Accord.Math.Vector3> relativePoints1 = CalculateRelativePoints(points1, centroid1);
        List<Accord.Math.Vector3> relativePoints2 = CalculateRelativePoints(points2, centroid2);

        // Create matrices for points
        Accord.Math.Matrix3x3 matrix1 = CreateMatrixFromPoints(relativePoints1);
        Accord.Math.Matrix3x3 matrix2 = CreateMatrixFromPoints(relativePoints2);

        // Compute the transformation matrix H
        Accord.Math.Matrix3x3 H = matrix1 * matrix2.Transpose();

        // Perform Singular Value Decomposition (SVD) on H
        H.SVD(out Accord.Math.Matrix3x3 U, out Accord.Math.Vector3 S, out Accord.Math.Matrix3x3 V);

        // Calculate the rotation matrix R
        Accord.Math.Matrix3x3 R = V * U.Transpose();

        // Ensure that the determinant of R is positive
        if (Determinant(R) < 0)
        {
            R.SVD(out Accord.Math.Matrix3x3 U_temp, out Accord.Math.Vector3 S_temp, out Accord.Math.Matrix3x3 V_temp);
            V_temp.V20 = -V_temp.V00;
            V_temp.V21 = -V_temp.V01;
            V_temp.V22 = -V_temp.V02;
            R = V_temp * U_temp.Transpose();
        }

        // Calculate the scale factor
        float scale_temp = (S.X + S.Y + S.Z) / 3f;

        // Calculate the translation vector
        translation = centroid2 - R * centroid1;

        // Set the output parameters
        rotation = R;
        scale = scale_temp;
    }

    Accord.Math.Vector3 CalculateCentroid(List<Vector3> points)
    {
        float sumX = 0, sumY = 0, sumZ = 0;

        foreach (Vector3 point in points)
        {
            sumX += point.x;
            sumY += point.y;
            sumZ += point.z;
        }

        int count = points.Count;

        return new Accord.Math.Vector3(sumX / count, sumY / count, sumZ / count);
    }

    List<Accord.Math.Vector3> CalculateRelativePoints(List<Vector3> points, Accord.Math.Vector3 centroid)
    {
        List<Accord.Math.Vector3> relativePoints = new List<Accord.Math.Vector3>();

        foreach (Vector3 point in points)
        {
            relativePoints.Add(new Accord.Math.Vector3(point.x - centroid.X, point.y - centroid.Y, point.z - centroid.Z));
        }

        return relativePoints;
    }

    Accord.Math.Matrix3x3 CreateMatrixFromPoints(List<Accord.Math.Vector3> points)
    {
        Accord.Math.Matrix3x3 matrix = new Accord.Math.Matrix3x3();

        for (int i = 0; i < points.Count; i++)
        {
            matrix[i, 0] = points[i].X;
            matrix[i, 1] = points[i].Y;
            matrix[i, 2] = points[i].Z;
        }

        return matrix;
    }

    float Determinant(Accord.Math.Matrix3x3 m)
    {
        return m.V00 * (m.V11 * m.V22 - m.V12 * m.V21) - m.V01 * (m.V10 * m.V22 - m.V12 * m.V20) + m.V02 * (m.V10 * m.V21 - m.V11 * m.V20);
    }

    List<Vector3> ApplyTransformation(List<Vector3> points, Accord.Math.Vector3 translation, Accord.Math.Matrix3x3 rotation, float scale)
    {
        List<Vector3> transformedPoints = new List<Vector3>();
        foreach (Vector3 point in points)
        {
            Accord.Math.Vector3 transformedPoint = rotation * point + translation;
            transformedPoints.Add(transformedPoint);
        }
        return transformedPoints;
    }

    void ApplyTransformationToPointCloud(List<Vector3> points, Accord.Math.Vector3 translation, Accord.Math.Matrix3x3 rotation, float scale)
    {
        for (int i = 0; i < pointCloud2.transform.childCount; i++)
        {
            Transform childTransform = pointCloud2.transform.GetChild(i);
            Vector3 point = childTransform.position;

            // Apply the transformation
            Accord.Math.Vector3 transformedPoint = rotation * point + translation * scale;

            // Draw the transform line
            if (drawTransformLine)
            {
                DrawTransformLine(childTransform.position, transformedPoint);
            }

            // Transform the point cloud
            if (transformPointCloud)
            {
                childTransform.position = transformedPoint;
            }
        }
    }

    void DrawTransformLine(Vector3 originalPoint, Accord.Math.Vector3 transformedPoint)
    {
        GameObject lineObject = new GameObject("TransformLine");
        lineObject.transform.position = originalPoint;

        LineRenderer lineRenderer = lineObject.AddComponent<LineRenderer>();
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = Color.yellow;
        lineRenderer.endColor = Color.yellow;
        lineRenderer.startWidth = 0.2f;
        lineRenderer.endWidth = 0.2f;

        lineRenderer.SetPosition(0, originalPoint);
        lineRenderer.SetPosition(1, new Vector3(transformedPoint.X, transformedPoint.Y, transformedPoint.Z));
    }

    int CountOverlappingPoints(List<Vector3> points1, List<Vector3> points2, float threshold)
    {
        int overlappingPoints = 0;
        int[] overlappingPoints1 = new int[points1.Count];

        for (int i = 0; i < points1.Count; i++)
        {
            for (int j = 0; j < points2.Count; j++)
            {
                if (DistanceTwoVectors(points1[i], points2[j]) < threshold && overlappingPoints1[i] == 0)
                {
                    overlappingPoints++;
                    overlappingPoints1[i] = 1;
                    break;
                }
            }
        }

        return overlappingPoints;
    }

    float DistanceTwoVectors(Vector3 vector1, Vector3 vector2)
    {
        return Vector3.Distance(vector1, vector2);
    }
}
