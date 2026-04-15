Shader "Custom/QuadCloudColorOnly"
{
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing

            #include "UnityCG.cginc"

            StructuredBuffer<float3> vertexBuffer;
            StructuredBuffer<float4> colorBuffer;

            UNITY_INSTANCING_BUFFER_START(Props)
            UNITY_INSTANCING_BUFFER_END(Props)

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv     : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float4 color  : COLOR0;
            };

            v2f vert(appdata v, uint instanceID : SV_InstanceID)
            {
                v2f o;

                UNITY_SETUP_INSTANCE_ID(v);

                float3 worldPos = vertexBuffer[instanceID];
                float4 color = colorBuffer[instanceID];

                float4 localPos = v.vertex * 0.03;
                float4 worldVertex = float4(localPos.xyz + worldPos, 1.0);

                o.vertex = UnityObjectToClipPos(worldVertex);
                o.color = color;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
}
