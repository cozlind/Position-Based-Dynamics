using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class PBD : MonoBehaviour {

    public Vector3 gravity=Vector3.zero;
    public List<Constraints> consList;
    [Range(0f,1f)]
    public float damping;
    public float[] m;
    float[] w;
    Vector3 f = Vector3.one;
    Vector3[] newPos, velocities;
    public Transform[] vertices;
    public int solverIterations=10;
    void Start()
    {
       
        newPos = new Vector3[vertices.Length];
        velocities =new Vector3[vertices.Length];
        w = new float[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            velocities[i] = Vector3.zero;
            newPos[i] = vertices[i].position;
            w[i] = 1.0f/m[i];
        }
    }
    void Update()
    {
        for(int i = 1; i < vertices.Length; i++)
        {
            velocities[i]+=Time.deltaTime*w[i]* Force(vertices[i].position);
        }
        DampVelocities(velocities);
        for (int i = 0; i < vertices.Length; i++)
        {
            newPos[i] = vertices[i].position + Time.deltaTime * velocities[i];
        }
        ////for (int i = 0; i < vertices.Length; i++)
        ////{
        ////    GenerateCollisionConstraints(newPos[i] - vertices[i].position);
        ////}
        for (int i = 0; i < solverIterations; i++)
        {
            ProjectConstraints(newPos);
        }
        for (int i = 0; i < vertices.Length; i++)
        {
            velocities[i] = (newPos[i] - vertices[i].position) / Time.deltaTime;
            vertices[i].position = newPos[i];
        }
    }
    Vector3 Force(Vector3 p)
    {
        return gravity;
    }
    void DampVelocities(Vector3[] v)
    {
        Vector3 sumX = Vector3.zero, sumV = Vector3.zero;
        float sumM = 0;
        for (int i = 0; i < v.Length; i++)
        {
            sumX += vertices[i].position * m[i];
            sumV += v[i] * m[i];
            sumM += m[i];
        }
        Vector3 centerX = sumX / sumM;
        Vector3 centerV = sumV / sumM;

        Vector3 tempL = Vector3.zero;
        Matrix4x4 tempI = Matrix4x4.zero;
        for (int i = 0; i < v.Length; i++)
        {
            Vector3 r = vertices[i].position - centerX;
            tempL += Vector3.Cross(r, m[i] * v[i]);

            Matrix4x4 rm = new Matrix4x4();
            rm.SetColumn(0, r);
            tempI = MatrixAdd(tempI, MatrixMultiply( rm * rm.transpose,m[i]));
        }
        Vector3 w = tempI.inverse.MultiplyVector(tempL) ;
        for(int i = 0; i < v.Length; i++)
        {
            Vector3 r = vertices[i].position - centerX;
            Vector3 deltaV = centerV + Vector3.Cross(w, r) - v[i];
            v[i] += damping * deltaV;
        }

    }
    static Matrix4x4 MatrixAdd(Matrix4x4 a,Matrix4x4 b)
    {
        Matrix4x4 m = Matrix4x4.zero;
        m.m00 = a.m00 + b.m00;
        m.m01 = a.m01 + b.m01;
        m.m02 = a.m02 + b.m02;
        m.m10 = a.m10 + b.m10;
        m.m11 = a.m11 + b.m11;
        m.m12 = a.m12 + b.m12;
        m.m20 = a.m20 + b.m20;
        m.m21 = a.m21 + b.m21;
        m.m22 = a.m22 + b.m22;
        return m;
    }
    static Matrix4x4 MatrixMultiply(Matrix4x4 a, float b)
    {
        Matrix4x4 m = Matrix4x4.zero;
        m.m00 = a.m00 * b;
        m.m01 = a.m01 * b;
        m.m02 = a.m02 * b;
        m.m10 = a.m10 * b;
        m.m11 = a.m11 * b;
        m.m12 = a.m12 * b;
        m.m20 = a.m20 * b;
        m.m21 = a.m21 * b;
        m.m22 = a.m22 * b;
        return m;
    }

    void GenerateCollisionConstraints(Vector3 v)
    {

    }
    //project distance constraints
    void ProjectConstraints(Vector3[] p)
    {
        foreach (var constraint in consList)
        {
            int i1 = constraint.index1;
            int i2 = constraint.index2;
            Vector3 deltaP1 = -w[i1] / (w[i1] + w[i2]) * (Vector3.Magnitude(p[i1] - p[i2]) - constraint.dist) * (p[i1] - p[i2]) / Vector3.Magnitude(p[i1] - p[i2]);
            Vector3 deltaP2 = w[i2] / (w[i1] + w[i2]) * (Vector3.Magnitude(p[i1] - p[i2]) - constraint.dist) * (p[i1] - p[i2]) / Vector3.Magnitude(p[i1] - p[i2]);
            p[i1] += deltaP1;
            p[i2] += deltaP2;
        }
    }
    [System.Serializable]
    public class Constraints
    {
        public int index1, index2;
        public float dist;
        public bool type;
        public float k;
    }
    //[System.Serializable]
    //public class Constraints
    //{
    //    Vector3 n;//cardinality
    //    int[] indices;
    //    float k;//stiffness parameter
    //    bool constraintType;//true = equality
    //    public virtual float ConstraintFunc(Vector3 v) { return 0; }
    //}
    //[System.Serializable]
    //public class DistantConstaint : Constraints
    //{
    //    public float dist;
    //    public Vector3 consPos;
    //    public float ConstraintFunc(Vector3 vertices.position,Vector3 consPos)
    //    {
    //        return Vector3.Distance(vertices.position, consPos) - dist;
    //    }
    //}
}
