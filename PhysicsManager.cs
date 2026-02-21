using System.Numerics;
using Raylib_cs;

class PhysicsManager
{
    public List<Body> bodies = new List<Body>();
    public int dyn_body_count;

    private const float IGNORE_COLL_THRESHOLD = 0.001f;

    public void Add_Dyn_Body(Body body)
    {
        bodies.Insert(0, body);
        dyn_body_count++;
    }

    public void Add_Stat_Body(Body body)
    {
        bodies.Add(body);
    }

    public void Move(Vector2 pointer, Vector2 delta)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            Body body = bodies[i];
            if (!Point_In_Body(pointer, bodies[i]))
                continue;

            for (int j = 0; j < body.points.Length; j++)
            {
                body.points[j].pos += delta;
                body.points[j].vel = Vector2.Zero;
            }
            return;
        }
    }

    public void Update(float dt)
    {
        // Update AABB
        for (int i = 0; i < bodies.Count; i++)
            Update_AABB(bodies[i]);
        //
        // // Update Normals
        for (int i = 0; i < bodies.Count; i++)
            Update_Normals(bodies[i]);

        // solve collisions
        for (int i = 0; i < dyn_body_count; i++)
            for (int j = 0; j < bodies.Count; j++)
                if (i != j)
                    Solve_Collision(bodies[i], bodies[j]);

        // update skeleton 
        for (int i = 0; i < dyn_body_count; i++)
            Update_Skeleton(bodies[i]);

        // update springs
        for (int i = 0; i < dyn_body_count; i++)
            Update_Springs(bodies[i]);

        // integrate 
        for (int i = 0; i < dyn_body_count; i++)
            Integrate(bodies[i], dt);

        for (int i = 0; i < bodies.Count; i++)
            DrawBody(bodies[i]);
    }

    private static void Update_AABB(Body body)
    {
        Vector2 min = new Vector2(float.MaxValue, float.MaxValue);
        Vector2 max = new Vector2(float.MinValue, float.MinValue);

        for (int i = 0; i < body.points.Length; i++)
        {
            min = Vector2.Min(min, body.points[i].pos);
            max = Vector2.Max(max, body.points[i].pos);
        }

        body.min = min;
        body.max = max;
    }

    private static void Update_Normals(Body body)
    {
        for (int i = 0; i < body.outer_spring_count; i++)
        {
            Spring spring = body.springs[i];
            Vector2 a = body.points[spring.ind_a].pos;
            Vector2 b = body.points[spring.ind_b].pos;
            body.spr_normals[i] = Vector2.Normalize(new Vector2(a.Y - b.Y, b.X - a.X));
        }
    }

    private static bool AABB_Intersect(Body body_a, Body body_b)
    {
        return !(Vector2.LessThanAny(body_a.max, body_b.min) || Vector2.LessThanAny(body_b.max, body_a.min));
    }

    private static bool Point_In_Body(Vector2 pos, Body body)
    {
        if (Vector2.LessThanAny(pos, body.min) || Vector2.LessThanAny(body.max, pos))
            return false;

        int count = 0;
        for (int i = 0; i < body.outer_spring_count; i++)
        {
            Spring spring = body.springs[i];
            Vector2 a = body.points[spring.ind_a].pos;
            Vector2 b = body.points[spring.ind_b].pos;
            Vector2 norm = body.spr_normals[i];

            Vector2 dir = new Vector2(1, 0);
            float dot = Vector2.Dot(dir, norm);
            if (MathF.Abs(dot) < 0.00001f)
                continue;

            float t = Vector2.Dot(a - pos, norm) / dot;
            if (t < 0)
                continue;

            Vector2 hit = pos + dir * t;
            if (Vector2.LessThanAny(hit, Vector2.Min(a, b)) || Vector2.LessThanAny(Vector2.Max(a, b), hit))
                continue;

            count++;
        }

        return count % 2 == 1;
    }

    private static (int, float) Get_Closest_Spring(Vector2 pos, Body body)
    {
        float min_dist = float.MaxValue;
        int min_ind = -1;

        for (int i = 0; i < body.outer_spring_count; i++)
        {
            Spring spring = body.springs[i];
            Vector2 a = body.points[spring.ind_a].pos;
            Vector2 n = body.spr_normals[i];

            float dst = Vector2.Dot(pos - a, n);
            if (dst > 0)
                continue;

            dst = MathF.Abs(dst);
            if (dst > min_dist)
                continue;

            min_dist = dst;
            min_ind = i;
        }

        return (min_ind, min_dist);
    }

    private static void Solve_Collision(Body body_a, Body body_b)
    {
        if (!AABB_Intersect(body_a, body_b))
            return;

        for (int i = 0; i < body_a.points.Length; i++)
        {
            Point point = body_a.points[i];
            if (!Point_In_Body(point.pos, body_b))
                continue;

            (int spr_ind, float dist) = Get_Closest_Spring(point.pos, body_b);
            if (dist < IGNORE_COLL_THRESHOLD)
                continue;

            if (spr_ind < 0)
                throw new Exception();

            Vector2 norm = body_b.spr_normals[spr_ind];
            if (Vector2.Dot(norm, point.vel) > 0)
                continue;

            Spring spr = body_b.springs[spr_ind];
            Vector2 spr_vel = (body_b.points[spr.ind_a].vel + body_b.points[spr.ind_b].vel) * 0.5f;
            Vector2 rel_vel = point.vel - spr_vel;
            Vector2 tang = new Vector2(norm.Y, -norm.X);
            float frict = Get_Friction(body_a, body_b);

            float coll_force = (body_a.settings.stiffness + body_b.settings.stiffness) * dist;
            float frict_force = coll_force * frict * MathF.Sign(Vector2.Cross(norm, rel_vel));

            body_a.points[i].force += coll_force * norm + frict_force * tang;
        }
    }

    private static float Get_Friction(Body body_a, Body body_b)
    {
        float frict_a = body_a.settings.friction;
        float frict_b = body_b.settings.friction;
        return MathF.Sqrt(frict_a * frict_a + frict_b * frict_b);
    }

    private static void Update_Skeleton(Body body)
    {
        Vector2 centroid = Vector2.Zero;
        for (int i = 0; i < body.points.Length; i++)
            centroid += body.points[i].pos;
        centroid /= body.points.Length;

        float ang = 0.0f;
        for (int i = 0; i < body.points.Length; i++)
        {
            Vector2 old_dir = Vector2.Normalize(body.skel_points[i]);
            Vector2 new_dir = Vector2.Normalize(body.points[i].pos - centroid);
            ang += MathF.Atan2(Vector2.Cross(new_dir, old_dir), Vector2.Dot(new_dir, old_dir));
        }
        ang /= body.points.Length;

        float cos = MathF.Cos(ang);
        float sin = MathF.Sin(ang);

        for (int i = 0; i < body.points.Length; i++)
        {
            Vector2 old_pos = body.skel_points[i];
            old_pos = new Vector2(old_pos.X * cos - old_pos.Y * sin, old_pos.X * sin + old_pos.Y * cos);
            Vector2 new_pos = body.points[i].pos - centroid;
            Vector2 delta = old_pos - new_pos;
            body.points[i].force += body.settings.skel_stiffness * delta;
        }
    }

    private static void Update_Springs(Body body)
    {
        for (int i = 0; i < body.springs.Length; i++)
        {
            Spring spring = body.springs[i];
            Point point_a = body.points[spring.ind_a];
            Point point_b = body.points[spring.ind_b];

            Vector2 delta = Vector2.Normalize(point_b.pos - point_a.pos);
            float dist = (point_b.pos - point_a.pos).Length();

            float stiff_force = (dist - spring.rest_dist) * body.settings.stiffness;
            float damp_force = Vector2.Dot(delta, point_b.vel - point_a.vel) * body.settings.damping;
            Vector2 force = delta * (stiff_force + damp_force);

            point_a.force += force;
            point_b.force -= force;

            body.points[spring.ind_a] = point_a;
            body.points[spring.ind_b] = point_b;
        }
    }

    private static void Integrate(Body body, float dt)
    {
        float inv_mass = body.settings.point_inv_mass;
        Vector2 grav_vec = new Vector2(0.0f, 9.81f);

        for (int i = 0; i < body.points.Length; i++)
        {
            Point point = body.points[i];
            point.vel += dt * inv_mass * point.force;
            point.vel += grav_vec * dt;
            point.pos += point.vel * dt;
            point.force = Vector2.Zero;
            body.points[i] = point;
        }
    }

    private static void DrawBody(Body body)
    {
        for (int i = 0; i < body.springs.Length; i++)
        {
            Spring spring = body.springs[i];
            Vector2 a = body.points[spring.ind_a].pos;
            Vector2 b = body.points[spring.ind_b].pos;

            Raylib.DrawLineV(a, b, Color.Green);
        }
    }
}

