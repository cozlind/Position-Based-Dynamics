using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class ClothPBD : MonoBehaviour
{
    List<Vector3> vertices;
    List<Vector2> uvs;
    List<int> indices;
    public int width;
    public int height;
    int count;
    MeshFilter meshFilter;
    Mesh mesh;

    public bool bendConstraint = true;
    public bool colliderConstraint = true;

    public Vector3 gravity = Vector3.zero;
    [Range(0f, 1f)]
    public float damping;
    public int solverIterations = 10;
    public float[] m;

    float[] w;
    Vector3 f = Vector3.one;
    Vector3[] newPos, velocities;

    public List<StretchConstraints> streConsList;
    public List<BendConstraints> bendConsList;
    public List<CollisionConstraints> collConsList;
    [Range(0, 50)]
    public float stretchDist = 1;
    [Range(0, 1)]
    public float stretchStiff = 1;
    [Range(0, 360)]
    public float bendTheta = 45;
    [Range(0, 1)]
    public float bendStiff = 1;
    [Range(0, 1)]
    public float colliderHeight = 0.5f;

    [System.Serializable]
    public class StretchConstraints
    {
        public int index1, index2;
        public float dist;
        public bool type;
        public float k;
        public StretchConstraints(int i1, int i2, float d, bool b, float s)
        {
            index1 = i1;
            index2 = i2;
            dist = d;
            type = b;
            k = s;
        }
    }
    [System.Serializable]
    public class BendConstraints
    {
        public int index1, index2, index3, index4;
        public float theta;
        public bool type;
        public float k;
        public BendConstraints(int i1, int i2, int i3, int i4, float d, bool b, float s)
        {
            index1 = i1;
            index2 = i2;
            index3 = i3;
            index4 = i4;
            theta = d;
            type = b;
            k = s;
        }
    }
    [System.Serializable]
    public class CollisionConstraints
    {
        public int index1, index2, index3, index4;
        public float dist;
        public bool type;
        public float k;
    }
    void Start()
    {
        InitMesh();
        InitParameters();
        InitConstraints();
    }
    void InitConstraints()
    {
        streConsList = new List<StretchConstraints>();
        bendConsList = new List<BendConstraints>();
        for (int i = 0; i < height - 1; i++)
        {
            for (int j = 0; j < width - 1; j++)
            {
                streConsList.Add(new StretchConstraints(i * width + j, i * width + j + 1, stretchDist, true, stretchStiff));
                streConsList.Add(new StretchConstraints(i * width + j, (i + 1) * width + j, stretchDist, true, stretchStiff));
                bendConsList.Add(new BendConstraints(i * width + j + 1, (i + 1) * width + j, i * width + j, (i + 1) * width + j + 1, bendTheta, true, bendStiff));
                bendConsList.Add(new BendConstraints(i * width + j, (i + 1) * width + j + 1, i * width + j + 1, (i + 1) * width + j, bendTheta, true, bendStiff));
            }
        }
    }
    void InitParameters()
    {
        newPos = new Vector3[count];
        velocities = new Vector3[count];
        m = new float[count];
        w = new float[count];
        for (int i = 0; i < count; i++)
        {
            velocities[i] = Vector3.zero;
            newPos[i] = vertices[i];
            if (i >= width * (height / 2 - 2) + width / 2 - 2 && i <= width * (height / 2 - 2) + width / 2 + 2) m[i] = 10000;
            else if (i >= width * (height / 2 - 1) + width / 2 - 2 && i <= width * (height / 2 - 1) + width / 2 + 2) m[i] = 10000;
            else if (i >= width * (height / 2) + width / 2 - 2 && i <= width * (height / 2) + width / 2 + 2) m[i] = 10000;
            else if (i >= width * (height / 2 + 1) + width / 2 - 2 && i <= width * (height / 2 + 1) + width / 2 + 2) m[i] = 10000;
            else if (i >= width * (height / 2 + 2) + width / 2 - 2 && i <= width * (height / 2 + 2) + width / 2 + 2) m[i] = 10000;
            else m[i] = 1;
            w[i] = 1.0f / m[i];
        }
    }
    void InitMesh()
    {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;

        count = width * height;
        vertices = new List<Vector3>();
        uvs = new List<Vector2>();
        indices = new List<int>();


        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                vertices.Add(new Vector3(-width / 2 + j + .5f, 0, -height / 2 + i + .5f));
                uvs.Add(new Vector2(j, i));
            }
        }
        for (int i = 0; i < height - 1; i++)
        {
            for (int j = 0; j < width - 1; j++)
            {
                indices.Add(width * i + j);
                indices.Add(width * (i + 1) + j);
                indices.Add(width * (i + 1) + j + 1);

                indices.Add(width * i + j);
                indices.Add(width * (i + 1) + j + 1);
                indices.Add(width * i + j + 1);
            }
        }
    }
    void UpdateMesh()
    {
        mesh.SetVertices(vertices);
        mesh.SetUVs(0, uvs);
        int[] x = indices.ToArray();
        mesh.SetIndices(x, MeshTopology.Triangles, 0);
        mesh.RecalculateNormals();
    }
    void Update()
    {
        for (int i = 1; i < count; i++)
        {
            velocities[i] += Time.deltaTime * w[i] * Force(vertices[i]);
        }
        DampVelocities(velocities);
        for (int i = 0; i < count; i++)
        {
            newPos[i] = vertices[i] + Time.deltaTime * velocities[i];
        }
        for (int i = 0; i < count; i++)
        {
            GenerateCollisionConstraints(newPos[i] - vertices[i]);
        }
        for (int i = 0; i < solverIterations; i++)
        {
            SolverStretchConstraint(newPos);
            if(bendConstraint) SolverBendConstraint(newPos);
        }
        for (int i = 0; i < count; i++)
        {
            velocities[i] = (newPos[i] - vertices[i]) / Time.deltaTime;
            vertices[i] = newPos[i];
        }
        UpdateMesh();
        //fixed the position
        transform.position = -vertices[width * (height / 2) + width / 2];
    }
    void GenerateCollisionConstraints(Vector3 v)
    {

    }
    //void SolverBendConstraint(Vector3[] p)
    //{
    //    foreach (var constraint in bendConsList)
    //    {
    //        int i1 = constraint.index1;
    //        int i2 = constraint.index2;
    //        int i3 = constraint.index3;
    //        int i4 = constraint.index4;

    //        Vector3 e = p[i4] - p[i3];
    //        float elen = e.magnitude;
    //        if (elen < 0.000001f) return;

    //        float invElen = 1.0f / elen;

    //        Vector3 n1 = Vector3.Cross(p[i3] - p[i1], p[i4] - p[i1]);
    //        n1 /= n1.sqrMagnitude;
    //        Vector3 n2 = Vector3.Cross(p[i4] - p[i2], p[i3] - p[i2]);
    //        n2 /= n2.sqrMagnitude;

    //        Vector3 d0 = elen * n1;
    //        Vector3 d1 = elen * n2;
    //        Vector3 d2 = Vector3.Dot(p[i1] - p[i4], e) * invElen * n1 + Vector3.Dot(p[i2] - p[i4], e) * invElen * n2;
    //        Vector3 d3 = Vector3.Dot(p[i3] - p[i1], e) * invElen * n1 + Vector3.Dot(p[i3] - p[i2], e) * invElen * n2;

    //        n1.Normalize();
    //        n2.Normalize();
    //        float dot = Vector3.Dot(n1, n2);

    //        if (dot < -1.0f) dot = -1.0f;
    //        if (dot > 1.0f) dot = 1.0f;
    //        float phi = Mathf.Rad2Deg * Mathf.Acos(dot);

    //        // float phi = (-0.6981317 * dot * dot - 0.8726646) * dot + 1.570796;	// fast approximation

    //        float lambda = w[0] * d0.sqrMagnitude + w[1] * d1.sqrMagnitude + w[2] * d2.sqrMagnitude + w[3] * d3.sqrMagnitude;

    //        if (lambda == 0.0) return;

    //        // stability
    //        // 1.5 is the largest magic number I found to be stable in all cases :-)
    //        //if (stiffness > 0.5 && fabs(phi - b.restAngle) > 1.5)		
    //        //	stiffness = 0.5;

    //        lambda = (phi - bendTheta) / lambda * bendStiff;

    //        if (Vector3.Dot(Vector3.Cross(n1, n2), e) > 0)
    //            lambda = -lambda;

    //        p[0] += -w[0] * lambda * d0;
    //        p[1] += -w[1] * lambda * d1;
    //        p[2] += -w[2] * lambda * d2;
    //        p[3] += -w[3] * lambda * d3;
    //    }
    //}
    /////////////////
    void SolverBendConstraint(Vector3[] p)
    {
        foreach (var constraint in bendConsList)
        {
            int i1 = constraint.index1;
            int i2 = constraint.index2;
            int i3 = constraint.index3;
            int i4 = constraint.index4;
            Vector3[] x = { p[i3], p[i4], p[i1], p[i2] };

            Vector3 e0 = x[1] - x[0];
            Vector3 e1 = x[2] - x[0];
            Vector3 e2 = x[3] - x[0];
            Vector3 e3 = x[2] - x[1];
            Vector3 e4 = x[3] - x[1];

            float c01 = Cot(e0, e1);
            float c02 = Cot(e0, e2);
            float c03 = Cot(-e0, e3);
            float c04 = Cot(-e0, e4);
            float a0 = 0.5f * Vector3.Cross(e0, e1).magnitude;
            float a1 = 0.5f * Vector3.Cross(e0, e2).magnitude;
            float coef = -3f / (2f * (a0 + a1));
            float[] k = { c03 + c04, c01 + c02, -c01 - c03, -c03 - c04 };
            float[] k2 = { coef * k[0], coef * k[1], coef * k[2], coef * k[3] };

            Matrix4x4 m = Matrix4x4.zero;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < i; j++)
                {
                    m[i, j] = m[j, i] = k[i] * k2[j];
                }
                m[i, i] = k[i] * k2[i];
            }

            float[] invMass = { w[i3], w[i4], w[i1], w[i2] };
            float energy = 0.0f;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    energy += m[j, i] * Vector3.Dot(x[i], x[j]);
                }
            }
            energy *= 0.5f;

            Vector3[] grad = { Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero };
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    grad[j] += m[j, i] * x[i];
                }
            }

            float sumNormGradC = 0;
            for (int i = 0; i < 4; i++)
            {
                if (invMass[i] != 0)
                    sumNormGradC += invMass[i] * grad[i].sqrMagnitude;
            }

            float eps = 0.000001f;
            if (Mathf.Abs(sumNormGradC) > eps)
            {
                float s = energy / sumNormGradC;
                p[i1] += -bendStiff * (s * invMass[2]) * grad[2];
                p[i2] += -bendStiff * (s * invMass[3]) * grad[3];
                p[i3] += -bendStiff * (s * invMass[0]) * grad[0];
                p[i4] += -bendStiff * (s * invMass[1]) * grad[1];
            }

        }
    }
    //////////////////
    //void SolverBendConstraint(Vector3[] p)
    //{
    //    foreach (var constraint in bendConsList)
    //    {
    //        int i1 = constraint.index1;
    //        int i2 = constraint.index2;
    //        int i3 = constraint.index3;
    //        int i4 = constraint.index4;
    //        Vector3 n1 = Vector3.Cross(p[i2], p[i3]).normalized;
    //        Vector3 n2 = Vector3.Cross(p[i2], p[i4]).normalized;
    //        float d = Vector3.Dot(n1, n2);
    //        Vector3 q3 = (Vector3.Cross(p[i2], n2) + Vector3.Cross(n1, p[i2]) * d) / Vector3.Cross(p[i2], p[i3]).magnitude;
    //        Vector3 q4 = (Vector3.Cross(p[i2], n1) + Vector3.Cross(n2, p[i2]) * d) / Vector3.Cross(p[i2], p[i4]).magnitude;
    //        Vector3 q2 = -(Vector3.Cross(p[i3], n2) + Vector3.Cross(n1, p[i3]) * d) / Vector3.Cross(p[i2], p[i3]).magnitude
    //            - (Vector3.Cross(p[i4], n1) + Vector3.Cross(n2, p[i4]) * d) / Vector3.Cross(p[i2], p[i4]).magnitude;
    //        Vector3 q1 = -q2 - q3 - q4;

    //        float sumwq = w[i1] * Vector3.SqrMagnitude(q1) + w[i2] * Vector3.SqrMagnitude(q2) + w[i3] * Vector3.SqrMagnitude(q3) + w[i4] * Vector3.SqrMagnitude(q4);
    //        Vector3 deltaP1 = -w[i1] * Mathf.Sqrt(1 - d * d) * (Mathf.Rad2Deg * Mathf.Acos(d) - bendTheta) * q1 / sumwq;
    //        Vector3 deltaP2 = -w[i2] * Mathf.Sqrt(1 - d * d) * (Mathf.Rad2Deg * Mathf.Acos(d) - bendTheta) * q2 / sumwq;
    //        Vector3 deltaP3 = -w[i3] * Mathf.Sqrt(1 - d * d) * (Mathf.Rad2Deg * Mathf.Acos(d) - bendTheta) * q3 / sumwq;
    //        Vector3 deltaP4 = -w[i4] * Mathf.Sqrt(1 - d * d) * (Mathf.Rad2Deg * Mathf.Acos(d) - bendTheta) * q4 / sumwq;

    //        p[i1] += deltaP1 * bendStiff;
    //        p[i2] += deltaP2 * bendStiff;
    //        p[i3] += deltaP3 * bendStiff;
    //        p[i4] += deltaP4 * bendStiff;

    //    }
    //}
    //void SolverTrianglePointDistanceConstraint(Vector3[] p)
    //{
    //    foreach (var constraint in bendConsList)
    //    {
    //            int i1 = constraint.index1;
    //            int i2 = constraint.index2;
    //            int i3 = constraint.index3;
    //            int i4 = constraint.index4;

    //            float b0 = 1.0f / 3.0f;        // for singular case
    //        float b1 = b0;
    //        float b2 = b0;

    //        Vector3 d1 = p[i3] - p[i2];
    //        Vector3 d2 = p[i4] - p[i2];
    //        Vector3 pp0 = p[i1] - p[i2];
    //        float a = Vector3.Dot(d1,d1);
    //        float b = Vector3.Dot(d2,d1);
    //        float c = Vector3.Dot(pp0,d1);
    //        float d = b;
    //        float e = Vector3.Dot(d2,d2);
    //        float f = Vector3.Dot(pp0,d2);
    //        float det = a * e - b * d;

    //        if (det != 0.0)
    //        {
    //            float s = (c * e - b * f) / det;
    //            float t = (a * f - c * d) / det;
    //            b0 = 1.0f - s - t;       // inside triangle
    //            b1 = s;
    //            b2 = t;
    //            if (b0 < 0.0)
    //            {       // on edge 1-2
    //                Vector3 d = p[i4] - p[i3];
    //                float d2 = Vector3.Dot(d,d);
    //                float t = (d2 == 0.0) ? 0.5 : Vector3.Dot(d,p[i1] - p[i3]) / d2;
    //                if (t < 0.0) t = 0.0;   // on point 1
    //                if (t > 1.0) t = 1.0;   // on point 2
    //                b0 = 0.0f;
    //                b1 = (1.0 - t);
    //                b2 = t;
    //            }
    //            else if (b1 < 0.0)
    //            {   // on edge 2-0
    //                Vector3 d = p[i2] - p[i4];
    //                float d2 = Vector3.Dot(d,d);
    //                float t = (d2 == 0.0) ? 0.5 : Vector3.Dot(d,p[i1] - p[i4]) / d2;
    //                if (t < 0.0) t = 0.0f;   // on point 2
    //                if (t > 1.0) t = 1.0f; // on point 0
    //                b1 = 0.0;
    //                b2 = (1.0f - t);
    //                b0 = t;
    //            }
    //            else if (b2 < 0.0)
    //            {   // on edge 0-1
    //                Vector3 d = p[i3] - p[i2];
    //                float d2 = d,d);
    //                float t = (d2 == 0.0) ? 0.5 : d,p - p[i2]) / d2;
    //                if (t < 0.0) t = 0.0;   // on point 0
    //                if (t > 1.0) t = 1.0;   // on point 1
    //                b2 = 0.0;
    //                b0 = (1.0 - t);
    //                b1 = t;
    //            }
    //        }
    //        Vector3 q = p[i2] * b0 + p[i3] * b1 + p[i4] * b2;
    //        Vector3 n = p - q;
    //        float dist = n.norm();
    //        n.normalize();
    //        float C = dist - restDist;
    //        Vector3 grad = n;
    //        Vector3 grad0 = -n * b0;
    //        Vector3 grad1 = -n * b1;
    //        Vector3 grad2 = -n * b2;

    //        float s = invMass + invMass0 * b0 * b0 + invMass1 * b1 * b1 + invMass2 * b2 * b2;
    //        if (s == 0.0)
    //            return false;

    //        s = C / s;
    //        if (C < 0.0)
    //            s *= compressionStiffness;
    //        else
    //            s *= stretchStiffness;

    //        if (s == 0.0)
    //            return false;

    //        corr = -s * invMass * grad;
    //        corr0 = -s * invMass0 * grad0;
    //        corr1 = -s * invMass1 * grad1;
    //        corr2 = -s * invMass2 * grad2;
    //    }
    //}


    void SolverStretchConstraint(Vector3[] p)
    {
        foreach (var constraint in streConsList)
        {
            int i1 = constraint.index1;
            int i2 = constraint.index2;
            Vector3 deltaP1 = -w[i1] / (w[i1] + w[i2]) * (Vector3.Magnitude(p[i1] - p[i2]) - constraint.dist) * (p[i1] - p[i2]) / Vector3.Magnitude(p[i1] - p[i2]);
            Vector3 deltaP2 = w[i2] / (w[i1] + w[i2]) * (Vector3.Magnitude(p[i1] - p[i2]) - constraint.dist) * (p[i1] - p[i2]) / Vector3.Magnitude(p[i1] - p[i2]);
            p[i1] += deltaP1 * stretchStiff;
            p[i2] += deltaP2 * stretchStiff;
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
            sumX += vertices[i] * m[i];
            sumV += v[i] * m[i];
            sumM += m[i];
        }
        Vector3 centerX = sumX / sumM;
        Vector3 centerV = sumV / sumM;

        Vector3 tempL = Vector3.zero;
        Matrix4x4 tempI = Matrix4x4.zero;
        for (int i = 0; i < v.Length; i++)
        {
            Vector3 r = vertices[i] - centerX;
            tempL += Vector3.Cross(r, m[i] * v[i]);

            Matrix4x4 rm = new Matrix4x4();
            rm.SetColumn(0, r);
            tempI = MatrixAdd(tempI, MatrixMultiply(rm * rm.transpose, m[i]));
        }
        Vector3 w = tempI.inverse.MultiplyVector(tempL);
        for (int i = 0; i < v.Length; i++)
        {
            Vector3 r = vertices[i] - centerX;
            Vector3 deltaV = centerV + Vector3.Cross(w, r) - v[i];
            v[i] += damping * deltaV;
        }

    }
    static Matrix4x4 MatrixAdd(Matrix4x4 a, Matrix4x4 b)
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
    
    float Cot(Vector3 v,Vector3 w)
    {
        float cos = Vector3.Dot(v, w);
        float sin = Vector3.Cross(v, w).magnitude;
        return cos / sin;
    }
}
