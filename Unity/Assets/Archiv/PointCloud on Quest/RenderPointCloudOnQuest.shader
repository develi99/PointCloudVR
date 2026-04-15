Shader "Custom/RenderQuadQuestInstanced"
{
    Properties
    {
        _PointSize("Point Size", Float) = 0.03
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 3.0
            #include "UnityCG.cginc"

            StructuredBuffer<float3> vertexBuffer; // Instanz-Zentren
            StructuredBuffer<float4> colorBuffer;  // Instanz-Farben

            float _PointSize;

            struct appdata
            {
                float3 vertex : POSITION;   // Vertex-Position im Mesh (Quad lokal)
                uint instanceID : SV_InstanceID;  // Instanzindex
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
            };

            v2f vert(appdata v)
            {
                v2f o;

                // Instanz-Position aus Buffer holen
                float3 center = vertexBuffer[v.instanceID];
                float4 color = colorBuffer[v.instanceID];

                // Lokale Vertexposition des Quads skalieren mit _PointSize
                float3 localPos = v.vertex*_PointSize;

                // Weltposition = Center + lokaler Offset
                float3 worldPos = center + localPos;

                // In Clipspace transformieren
                o.pos = UnityObjectToClipPos(float4(worldPos, 1.0));
                o.color = color;

                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                return i.color; // Nur zum Testen

            }/*
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 3.0

            StructuredBuffer<float3> vertexBuffer;
            StructuredBuffer<float4> colorBuffer;

            float _PointSize;
            //float4x4 unity_MatrixVP;

            struct appdata
            {
                float3 vertex : POSITION;
                uint instanceID : SV_InstanceID;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
            };

            v2f vert(appdata v)
            {
                v2f o;

                float3 center = vertexBuffer[v.instanceID];
                float4 color = colorBuffer[v.instanceID];
                float3 localPos = v.vertex * _PointSize;

                float3 worldPos = center + localPos;

                o.pos = mul(unity_MatrixVP, float4(worldPos, 1.0));
                o.color = color;

                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                return i.color;
            }
            */
            ENDCG
        }
    }
}
