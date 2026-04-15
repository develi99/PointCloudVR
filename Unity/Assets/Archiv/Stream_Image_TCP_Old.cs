using UnityEngine;
using System.Net.Sockets;

public class SimpleTcpCameraClient : MonoBehaviour
{
    [Header("TCP Settings")]
    public string host = "192.168.0.208";
    public int port = 9999;

    [Header("Frame Settings")]
    public int width = 640;
    public int height = 480;

    private TcpClient client;
    private NetworkStream stream;
    private Texture2D texture;
    private Renderer rend;

    void Start()
    {
        rend = GetComponent<Renderer>();
        texture = new Texture2D(width, height, TextureFormat.RGB24, false);
        rend.material.mainTexture = texture;

        // Sofort verbinden (blockierend)
        client = new TcpClient(host, port);
        stream = client.GetStream();
        Debug.Log($"[TCP] Verbunden mit {host}:{port}");
    }

    void Update()
    {
        // Wenn keine Daten oder keine Verbindung, nichts tun
        if (stream == null || !stream.DataAvailable) return;

        // 1) 4-Byte-Länge lesen
        byte[] lenBytes = new byte[4];
        int read = stream.Read(lenBytes, 0, 4);
        if (read < 4) return;

        int frameLen =
            (lenBytes[0] << 24) |
            (lenBytes[1] << 16) |
            (lenBytes[2] << 8) |
             lenBytes[3];

        // 2) JPEG-Daten holen
        byte[] jpg = new byte[frameLen];
        int offset = 0;
        while (offset < frameLen)
        {
            int chunk = stream.Read(jpg, offset, frameLen - offset);
            if (chunk <= 0) return;  // Verbindung verloren
            offset += chunk;
        }

        // 3) Als Texture anwenden
        texture.LoadImage(jpg);
        texture.Apply();
    }

    void OnDestroy()
    {
        stream?.Close();
        client?.Close();
    }
}
