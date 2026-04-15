using UnityEngine;
using System;
using System.Net.Sockets;

public class AutoReconnectTcpClient : MonoBehaviour
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
    private bool connected = false;

    void Start()
    {
        rend = GetComponent<Renderer>();
        texture = new Texture2D(width, height, TextureFormat.RGB24, false);
        rend.material.mainTexture = texture;
    }

    void Update()
    {
        // 1) Wenn noch nicht verbunden, versuche es
        if (!connected)
        {
            try
            {
                client = new TcpClient(host, port);
                stream = client.GetStream();
                connected = true;
                Debug.Log($"[TCP] 🎉 Verbunden mit {host}:{port}");
            }
            catch (Exception e)
            {
                Debug.Log($"[TCP] noch nicht verbunden, nächster Versuch... ({e.Message})");
                return;
            }
        }

        // 2) Wenn verbunden und Daten da sind, lese ein Frame
        if (stream.DataAvailable)
        {
            // 2.1 Länge lesen (blockierend, bis 4 Byte da sind)
            byte[] lenBytes = new byte[4];
            int got = stream.Read(lenBytes, 0, 4);
            if (got < 4) return;

            int frameLen =
                (lenBytes[0] << 24) | (lenBytes[1] << 16) |
                (lenBytes[2] << 8) | lenBytes[3];

            // 2.2 Frame-Bytes lesen
            byte[] jpg = new byte[frameLen];
            int offset = 0;
            while (offset < frameLen)
            {
                int chunk = stream.Read(jpg, offset, frameLen - offset);
                if (chunk <= 0) return;  // Verbindung abgebrochen
                offset += chunk;
            }

            // 2.3 Als Texture anzeigen
            texture.LoadImage(jpg);
            texture.Apply();
        }
    }

    void OnDestroy()
    {
        stream?.Close();
        client?.Close();
    }
}
