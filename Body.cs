using System.Numerics;

class Body
{
    public Settings settings;
    public Point[] points;
    public Spring[] springs;
    public int outer_spring_count;

    public Vector2 min, max;
    public Vector2[] skel_points;
    public Vector2[] spr_normals;

    public Body(Settings settings, Point[] points, Spring[] springs, int outer_spring_count)
    {
        this.settings = settings;
        this.points = points;
        this.springs = springs;
        this.outer_spring_count = outer_spring_count;

        Vector2 centroid = Vector2.Zero;
        for (int i = 0; i < points.Length; i++)
            centroid += points[i].pos;
        centroid /= points.Length;

        skel_points = new Vector2[points.Length];
        for (int i = 0; i < points.Length; i++)
            skel_points[i] = points[i].pos - centroid;

        // нормаль соответствует внешней струне
        spr_normals = new Vector2[outer_spring_count];
    }
}

