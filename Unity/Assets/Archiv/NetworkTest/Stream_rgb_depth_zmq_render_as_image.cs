using UnityEngine;
using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

public class ZmqDepthClient : MonoBehaviour
{
    public Renderer rgbRenderer;
    public Renderer depthRenderer;

    private Texture2D rgbTexture;
    private Texture2D depthTexture;

    private ConcurrentQueue<byte[]> rgbQueue = new ConcurrentQueue<byte[]>();
    private ConcurrentQueue<byte[]> depthQueue = new ConcurrentQueue<byte[]>();

    private Thread listenerThread;
    private bool isRunning = false;
    private PullSocket subSocket;

    void Start()
    {
        rgbTexture = new Texture2D(640, 480, TextureFormat.RGB24, false);
        depthTexture = new Texture2D(640, 480, TextureFormat.RGB24, false);

        rgbRenderer.material.mainTexture = rgbTexture;
        depthRenderer.material.mainTexture = depthTexture;

        isRunning = true;
        listenerThread = new Thread(ZmqListener);
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    void Update()
    {
        if (rgbQueue.TryDequeue(out byte[] rgbBytes))
        {
            rgbTexture.LoadImage(rgbBytes);
            rgbTexture.Apply();
        }

        if (depthQueue.TryDequeue(out byte[] depthBytes))
        {
            depthTexture.LoadImage(depthBytes);
            depthTexture.Apply();
        }
    }

    void OnDestroy()
    {
        isRunning = false;

        if (subSocket != null)
        {
            try
            {
                subSocket.Close();
                subSocket.Dispose();
            }
            catch (Exception e)
            {
                Debug.LogWarning("Error closing socket: " + e);
            }
        }

        if (listenerThread != null && listenerThread.IsAlive)
        {
            listenerThread.Join(500); // Max 0.5s warten
        }

        NetMQConfig.Cleanup(); // Wichtig, um Hintergrund-Threads von NetMQ zu schließen
    }

    void ZmqListener()
    {
        AsyncIO.ForceDotNet.Force();

        using (subSocket = new PullSocket(">tcp://localhost:5555"))
        {
            while (isRunning)
            {
                try
                {
                    if (subSocket.TryReceiveFrameBytes(TimeSpan.FromMilliseconds(100), out byte[] msg))
                    {
                        if (msg.Length < 8)
                        {
                            Debug.LogWarning("[ZMQ] Nachricht zu kurz für Header");
                            continue;
                        }

                        int rgbLen = BitConverter.ToInt32(msg, 0);
                        if (msg.Length < 4 + rgbLen + 4) continue;

                        byte[] rgbBytes = new byte[rgbLen];
                        Buffer.BlockCopy(msg, 4, rgbBytes, 0, rgbLen);

                        int depthLen = BitConverter.ToInt32(msg, 4 + rgbLen);
                        if (msg.Length < 4 + rgbLen + 4 + depthLen) continue;

                        byte[] depthBytes = new byte[depthLen];
                        Buffer.BlockCopy(msg, 4 + rgbLen + 4, depthBytes, 0, depthLen);

                        rgbQueue.Enqueue(rgbBytes);
                        depthQueue.Enqueue(depthBytes);
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogError("[ZMQ] Fehler im Listener: " + ex.Message);
                    break;
                }
            }
        }
    }
}
