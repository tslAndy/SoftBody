using System.Numerics;
using Raylib_cs;

class Program
{
    public static void Main()
    {
        Raylib.InitWindow(1000, 500, "Hello world!");

        PhysicsManager manager = new PhysicsManager();

        float d = 40;
        float h = 500;
        float w = 1000;
        manager.Add_Stat_Body(Create_Obstacle(
            new Vector2(0, d), new Vector2(0, h - d), new Vector2(d, h - d), new Vector2(d, d)
        ));
        manager.Add_Stat_Body(Create_Obstacle(
            new Vector2(0, h - d), new Vector2(0, h), new Vector2(w, h), new Vector2(w, h - d)
        ));
        manager.Add_Stat_Body(Create_Obstacle(
            new Vector2(w - d, h - d), new Vector2(w, h - d), new Vector2(w, d), new Vector2(w - d, d)
        ));
        manager.Add_Stat_Body(Create_Obstacle(
            new Vector2(0, 0), new Vector2(0, d), new Vector2(w, d), new Vector2(w, 0)
        ));

        manager.Add_Stat_Body(Create_Obstacle(
            new Vector2(40, h - d),
            new Vector2(230, h - d),
            new Vector2(230, 350),
            new Vector2(40, 300)
        ));

        manager.Add_Dyn_Body(Create_Rectangle(10, 10, 10.0f, new Vector2(300, 50)));
        manager.Add_Dyn_Body(Create_Rectangle(10, 10, 10.0f, new Vector2(100, 50)));
        manager.Add_Dyn_Body(Create_Circle_Body(20, 50.0f, new Vector2(400, 200)));
        manager.Add_Dyn_Body(Create_Circle_Body(20, 50.0f, new Vector2(420, 400)));


        while (!Raylib.WindowShouldClose())
        {
            Raylib.BeginDrawing();
            Raylib.ClearBackground(Color.Black);

            if (Raylib.IsMouseButtonDown(MouseButton.Left))
                manager.Move(Raylib.GetMousePosition(), Raylib.GetMouseDelta());
            manager.Update(Math.Clamp(Raylib.GetFrameTime(), 0.0f, 0.016f));

            Raylib.DrawFPS(10, 10);
            Raylib.EndDrawing();
        }

        Raylib.CloseWindow();
    }

    public static Body Create_Rectangle(int w, int h, float step, Vector2 offset)
    {
        Settings settings = new Settings
        {
            stiffness = 50f,
            damping = 10f,
            friction = 0.01f,
            point_inv_mass = 1.0f,
            skel_stiffness = 30f
        };

        int x, y;

        Point[] points = new Point[w * h];
        for (y = 0; y < h; y++)
            for (x = 0; x < w; x++)
                points[y * w + x] = new Point { pos = offset + new Vector2(x, y) * step };

        int springs_len = h * (w - 1) + w * (h - 1) + 2 * (w - 1) * (h - 1);
        Spring[] springs = new Spring[springs_len];

        int ind = 0;

        y = 0;
        for (x = 0; x < w - 1; x++)
            springs[ind++] = new Spring { ind_a = y * w + x + 1, ind_b = y * w + x, rest_dist = step };

        y = h - 1;
        for (x = 0; x < w - 1; x++)
            springs[ind++] = new Spring { ind_a = y * w + x, ind_b = y * w + x + 1, rest_dist = step };

        x = 0;
        for (y = 0; y < h - 1; y++)
            springs[ind++] = new Spring { ind_a = y * w + x, ind_b = (y + 1) * w + x, rest_dist = step };

        x = w - 1;
        for (y = 0; y < h - 1; y++)
            springs[ind++] = new Spring { ind_a = (y + 1) * w + x, ind_b = y * w + x, rest_dist = step };

        for (y = 0; y < h - 1; y++)
            for (x = 1; x < w - 1; x++)
                springs[ind++] = new Spring { ind_a = y * w + x, ind_b = (y + 1) * w + x, rest_dist = step };

        for (y = 1; y < h - 1; y++)
            for (x = 0; x < w - 1; x++)
                springs[ind++] = new Spring { ind_a = y * w + x, ind_b = y * w + x + 1, rest_dist = step };

        for (y = 0; y < h - 1; y++)
            for (x = 0; x < w - 1; x++)
                springs[ind++] = new Spring { ind_a = y * w + x, ind_b = (y + 1) * w + x + 1, rest_dist = step * 1.44444f };

        for (y = 0; y < h - 1; y++)
            for (x = 1; x < w; x++)
                springs[ind++] = new Spring { ind_a = y * w + x, ind_b = (y + 1) * w + x - 1, rest_dist = step * 1.44444f };

        return new Body(
            settings,
            points,
            springs,
            2 * (w - 1 + h - 1)
        );
    }

    public static Body Create_Circle_Body(int n, float radius, Vector2 offset)
    {
        Settings settings = new Settings
        {
            stiffness = 100f,
            damping = 100f,
            friction = 0.01f,
            point_inv_mass = 1.0f,
            skel_stiffness = 200f
        };

        Point[] points = new Point[n];

        float step = MathF.Tau / n;
        for (int i = 0; i < n; i++)
        {
            float ang = -i * step;
            float cos = MathF.Cos(ang);
            float sin = MathF.Sin(ang);
            points[i] = new Point { pos = offset + new Vector2(cos, sin) * radius };
        }

        float rest_dist = 2 * MathF.Sin(0.5f * step) * radius;

        Spring[] springs = new Spring[n];
        for (int i = 0; i < n; i++)
            springs[i] = new Spring { ind_a = i, ind_b = (i + 1) % n, rest_dist = rest_dist };

        return new Body(settings, points, springs, n);
    }

    public static Body Create_Obstacle(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        return new Body(
            new Settings
            {
                stiffness = 1000.0f,
                damping = 0.4f,
                friction = 0.1f,
                point_inv_mass = 1.0f
            },
            new Point[]
            {
                new Point { pos = a },
                new Point { pos = b },
                new Point { pos = c },
                new Point { pos = d },
            },
            new Spring[]
            {
                new Spring { ind_a = 0, ind_b = 1 },
                new Spring { ind_a = 1, ind_b = 2 },
                new Spring { ind_a = 2, ind_b = 3 },
                new Spring { ind_a = 3, ind_b = 0 },
            },
            outer_spring_count: 4
        );
    }
}
