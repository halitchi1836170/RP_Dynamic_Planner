using System.IO;
using System.Text;
using System.Globalization;
using UnityEngine;

// Salvataggio/lettura della occupancy grid in formato standard ROS map_server: .pgm (P5 binario) + .yaml.
// Convenzione dati: sbyte[] row-major, valori 0 (libera) .. 100 (occupata), -1 (sconosciuta).
// Convenzione pixel PGM (ROS): libera = 254 (bianco), occupata = 0 (nero), sconosciuta = 205 (grigio).
// NB: il PGM ha origine in alto-sinistra (y verso il basso), l'OccupancyGrid ROS in basso-sinistra
//     (y verso l'alto) -> le righe vengono invertite in y in scrittura e in lettura (round-trip coerente).
public class IOFileOperationService
{
    private readonly string pgmPath;
    private readonly string yamlPath;

    private const byte UNKNOWN_PIXEL = 205;

    public IOFileOperationService(string baseFileNameWithoutExt)
    {
        string dir = Application.persistentDataPath;
        pgmPath = Path.Combine(dir, baseFileNameWithoutExt + ".pgm");
        yamlPath = Path.Combine(dir, baseFileNameWithoutExt + ".yaml");
    }

    public string GetPgmPath() => pgmPath;
    public string GetYamlPath() => yamlPath;

    // --------------------------------------------------------------------------------------------
    //                                         SCRITTURA
    // --------------------------------------------------------------------------------------------
    public void WriteOccupancyGrid(sbyte[] data, int width, int height, float resolution, float originX, float originY)
    {
        if (data == null || data.Length != width * height || width <= 0 || height <= 0) return;

        // ---- PGM (P5 binario) ----
        byte[] header = Encoding.ASCII.GetBytes($"P5\n{width} {height}\n255\n");
        byte[] pixels = new byte[width * height];
        for (int y = 0; y < height; y++)
        {
            int srcRow = (height - 1 - y) * width;   // flip verticale: riga PGM y <- riga grid (height-1-y)
            int dstRow = y * width;
            for (int x = 0; x < width; x++)
            {
                sbyte occ = data[srcRow + x];
                byte v;
                if (occ < 0) v = UNKNOWN_PIXEL;                                       // sconosciuta -> grigio
                else v = (byte)Mathf.RoundToInt(254f * (100 - occ) / 100f);          // 0->254 (bianco), 100->0 (nero)
                pixels[dstRow + x] = v;
            }
        }
        using (FileStream fs = new FileStream(pgmPath, FileMode.Create))
        {
            fs.Write(header, 0, header.Length);
            fs.Write(pixels, 0, pixels.Length);
        }

        // ---- YAML ----
        CultureInfo ci = CultureInfo.InvariantCulture;
        StringBuilder yaml = new StringBuilder();
        yaml.Append("image: ").Append(Path.GetFileName(pgmPath)).Append('\n');
        yaml.Append("resolution: ").Append(resolution.ToString(ci)).Append('\n');
        yaml.Append("origin: [").Append(originX.ToString(ci)).Append(", ").Append(originY.ToString(ci)).Append(", 0.0]\n");
        yaml.Append("negate: 0\n");
        yaml.Append("occupied_thresh: 0.65\n");
        yaml.Append("free_thresh: 0.25\n");
        File.WriteAllText(yamlPath, yaml.ToString());
    }

    // --------------------------------------------------------------------------------------------
    //                                         LETTURA
    // --------------------------------------------------------------------------------------------
    // Ritorna l'occupancy grid come sbyte[W*H] (row-major, convenzione ROS: 0..100, -1 = sconosciuta).
    // I metadati (dimensioni, risoluzione, origine) sono restituiti via out.
    public (sbyte[], int, int, float, float, float) ReadOccupancyGrid()
    {
        int width = 0; int height = 0; float resolution = 0f; float originX = 0f; float originY = 0f;
        if (!File.Exists(pgmPath) || !File.Exists(yamlPath)) return (new sbyte[0], width, height, originX, originY, resolution);

        // ---- parse YAML (resolution, origin) ----
        CultureInfo ci = CultureInfo.InvariantCulture;
        foreach (string rawLine in File.ReadAllLines(yamlPath))
        {
            string line = rawLine.Trim();
            if (line.StartsWith("resolution:"))
            {
                resolution = float.Parse(line.Substring("resolution:".Length).Trim(), ci);
            }
            else if (line.StartsWith("origin:"))
            {
                int lb = line.IndexOf('[');
                int rb = line.IndexOf(']');
                string inside = line.Substring(lb + 1, rb - lb - 1);
                string[] parts = inside.Split(',');
                originX = float.Parse(parts[0].Trim(), ci);
                originY = float.Parse(parts[1].Trim(), ci);
            }
        }

        // ---- parse PGM (P5 binario) ----
        byte[] all = File.ReadAllBytes(pgmPath);
        int pos = 0;
        ReadToken(all, ref pos);                       // magic "P5"
        width = int.Parse(ReadToken(all, ref pos));
        height = int.Parse(ReadToken(all, ref pos));
        ReadToken(all, ref pos);                       // maxval (255)
        pos++;                                         // salta l'unico whitespace dopo maxval -> inizio dati binari

        sbyte[] data = new sbyte[width * height];
        for (int y = 0; y < height; y++)
        {
            int srcRow = y * width;                    // riga PGM
            int dstRow = (height - 1 - y) * width;     // flip verticale inverso -> riga grid
            for (int x = 0; x < width; x++)
            {
                byte v = all[pos + srcRow + x];
                sbyte occ;
                if (v == UNKNOWN_PIXEL) occ = -1;                                    // sconosciuta
                else occ = (sbyte)Mathf.RoundToInt(100f * (254 - v) / 254f);         // inverso della scrittura
                data[dstRow + x] = occ;
            }
        }
        return (data, width, height, originX, originY, resolution);
    }

    // Legge un token ASCII delimitato da whitespace da un buffer di byte (per l'header PGM).
    private string ReadToken(byte[] buf, ref int pos)
    {
        while (pos < buf.Length && (buf[pos] == ' ' || buf[pos] == '\n' || buf[pos] == '\r' || buf[pos] == '\t')) pos++;
        int start = pos;
        while (pos < buf.Length && !(buf[pos] == ' ' || buf[pos] == '\n' || buf[pos] == '\r' || buf[pos] == '\t')) pos++;
        return Encoding.ASCII.GetString(buf, start, pos - start);
    }
}
