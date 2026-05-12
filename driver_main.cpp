#include <openvr_driver.h>
#include <winsock2.h>
#include <vector>
#include <cstring>
#include <cmath>

// --- МАТЕМАТИКА ДЛЯ ШЛЕМА И СГЛАЖИВАНИЯ ---
vr::HmdQuaternion_t GetMatrixQuat(const vr::HmdMatrix34_t& m) {
    vr::HmdQuaternion_t q;
    q.w = sqrt(fmax(0.0f, 1.0f + m.m[0][0] + m.m[1][1] + m.m[2][2])) / 2.0f;
    q.x = sqrt(fmax(0.0f, 1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2])) / 2.0f;
    q.y = sqrt(fmax(0.0f, 1.0f - m.m[0][0] + m.m[1][1] - m.m[2][2])) / 2.0f;
    q.z = sqrt(fmax(0.0f, 1.0f - m.m[0][0] - m.m[1][1] + m.m[2][2])) / 2.0f;
    q.x = copysign(q.x, m.m[2][1] - m.m[1][2]);
    q.y = copysign(q.y, m.m[0][2] - m.m[2][0]);
    q.z = copysign(q.z, m.m[1][0] - m.m[0][1]);
    return q;
}

vr::HmdQuaternion_t MultiplyQuat(const vr::HmdQuaternion_t& q1, const vr::HmdQuaternion_t& q2) {
    vr::HmdQuaternion_t res;
    res.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    res.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    res.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    res.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return res;
}

// SLERP: Сферическая интерполяция кватернионов (Плавное вращение)
vr::HmdQuaternion_t SlerpQuat(const vr::HmdQuaternion_t& q1, const vr::HmdQuaternion_t& q2, float t) {
    float dot = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w;
    vr::HmdQuaternion_t q3 = q2;

    if (dot < 0.0f) {
        dot = -dot;
        q3.x = -q3.x; q3.y = -q3.y; q3.z = -q3.z; q3.w = -q3.w;
    }

    if (dot > 0.9995f) {
        vr::HmdQuaternion_t res;
        res.x = q1.x + t * (q3.x - q1.x);
        res.y = q1.y + t * (q3.y - q1.y);
        res.z = q1.z + t * (q3.z - q1.z);
        res.w = q1.w + t * (q3.w - q1.w);
        float mag = sqrt(res.x*res.x + res.y*res.y + res.z*res.z + res.w*res.w);
        res.x /= mag; res.y /= mag; res.z /= mag; res.w /= mag;
        return res;
    }

    float theta_0 = acos(dot);
    float theta = theta_0 * t;
    float sin_theta = sin(theta);
    float sin_theta_0 = sin(theta_0);

    float s0 = cos(theta) - dot * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;

    vr::HmdQuaternion_t res;
    res.x = s0 * q1.x + s1 * q3.x;
    res.y = s0 * q1.y + s1 * q3.y;
    res.z = s0 * q1.z + s1 * q3.z;
    res.w = s0 * q1.w + s1 * q3.w;
    return res;
}

// Lerp: Линейная интерполяция позиции (Плавное движение)
float Lerp(float start, float end, float t) {
    return start + t * (end - start);
}

// --- БИНАРНАЯ СТРУКТУРА ПАКЕТА ---
struct __attribute__((packed)) BoneQuat {
    float x, y, z, w;
};

struct __attribute__((packed)) HandPacket {
    uint8_t isRightHand;    
    float posX, posY, posZ; 
    BoneQuat bones[31];     
};

// --- КЛАСС КОНТРОЛЛЕРА ---
class CHandDevice : public vr::ITrackedDeviceServerDriver {
private:
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;
    vr::VRInputComponentHandle_t m_skeletonHandle;
    bool m_isRightHand;

    // --- ПЕРЕМЕННЫЕ ДЛЯ СГЛАЖИВАНИЯ ---
    bool m_isFirstUpdate = true;
    float m_targetPos[3] = {0, 0, 0};
    vr::HmdQuaternion_t m_targetQuat = {1, 0, 0, 0};
    
    float m_currentPos[3] = {0, 0, 0};
    vr::HmdQuaternion_t m_currentQuat = {1, 0, 0, 0};

    // Фактор интерполяции (0.0 - нет движения, 1.0 - мгновенный прыжок без сглаживания)
    // 0.15 отлично подходит для 144 Гц шлема и 30 Гц камеры.
    const float SMOOTH_FACTOR = 0.15f; 

public:
    CHandDevice(bool isRightHand) : m_isRightHand(isRightHand), m_unObjectId(vr::k_unTrackedDeviceIndexInvalid) {}

    virtual vr::EVRInitError Activate(uint32_t unObjectId) override {
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_isRightHand ? "PurpleHand_Right" : "PurpleHand_Left");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5"); 
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, m_isRightHand ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{purplehands}/icons/hand_off.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{purplehands}/icons/hand_searching.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{purplehands}/icons/hand_ready.png");

        const char* skeletonPath = m_isRightHand ? "/input/skeleton/right" : "/input/skeleton/left";
        const char* skeletonBase = m_isRightHand ? "/skeleton/hand/right" : "/skeleton/hand/left";
        
        vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, 
                                                     skeletonPath, skeletonBase, 
                                                     "/pose/raw", 
                                                     vr::VRSkeletalTracking_Partial, 
                                                     nullptr, 0, &m_skeletonHandle);

        return vr::VRInitError_None;
    }

    virtual void Deactivate() override { m_unObjectId = vr::k_unTrackedDeviceIndexInvalid; }
    virtual void EnterStandby() override {}
    virtual void* GetComponent(const char* pchComponentNameAndVersion) override { return nullptr; }
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override {}

    virtual vr::DriverPose_t GetPose() override {
        vr::DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = vr::TrackingResult_Running_OK;
        pose.deviceIsConnected = true;
        pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
        pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
        pose.qRotation = { 1, 0, 0, 0 };
        return pose;
    }

    // Сохраняем новые данные от Питона (Цель)
    void OnPacketReceived(const HandPacket& packet) {
        m_targetPos[0] = packet.posX;
        m_targetPos[1] = packet.posY;
        m_targetPos[2] = packet.posZ;
        
        m_targetQuat = { packet.bones[0].w, packet.bones[0].x, packet.bones[0].y, packet.bones[0].z };

        // Если это первый пакет, телепортируем руку мгновенно (чтобы она не летела из точки 0,0,0)
        if (m_isFirstUpdate) {
            m_currentPos[0] = m_targetPos[0]; m_currentPos[1] = m_targetPos[1]; m_currentPos[2] = m_targetPos[2];
            m_currentQuat = m_targetQuat;
            m_isFirstUpdate = false;
        }
    }

    // Вызывается 144 раза в секунду (Плавное обновление)
    void ProcessFrame() {
        if (m_unObjectId == vr::k_unTrackedDeviceIndexInvalid || m_isFirstUpdate) return;

        // 1. СГЛАЖИВАНИЕ ЛОКАЛЬНЫХ КООРДИНАТ
        m_currentPos[0] = Lerp(m_currentPos[0], m_targetPos[0], SMOOTH_FACTOR);
        m_currentPos[1] = Lerp(m_currentPos[1], m_targetPos[1], SMOOTH_FACTOR);
        m_currentPos[2] = Lerp(m_currentPos[2], m_targetPos[2], SMOOTH_FACTOR);
        
        m_currentQuat = SlerpQuat(m_currentQuat, m_targetQuat, SMOOTH_FACTOR);

        // 2. ПЕРЕВОД В ГЛОБАЛЬНЫЕ КООРДИНАТЫ ПО СВЕЖЕЙ ПОЗИЦИИ ШЛЕМА
        vr::TrackedDevicePose_t hmdPose;
        vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, &hmdPose, 1);
        const vr::HmdMatrix34_t& mat = hmdPose.mDeviceToAbsoluteTracking;

        float worldX = mat.m[0][0] * m_currentPos[0] + mat.m[0][1] * m_currentPos[1] + mat.m[0][2] * m_currentPos[2] + mat.m[0][3];
        float worldY = mat.m[1][0] * m_currentPos[0] + mat.m[1][1] * m_currentPos[1] + mat.m[1][2] * m_currentPos[2] + mat.m[1][3];
        float worldZ = mat.m[2][0] * m_currentPos[0] + mat.m[2][1] * m_currentPos[1] + mat.m[2][2] * m_currentPos[2] + mat.m[2][3];

        vr::HmdQuaternion_t hmdQuat = GetMatrixQuat(mat);
        vr::HmdQuaternion_t finalQuat = MultiplyQuat(hmdQuat, m_currentQuat);

        // 3. ОТПРАВКА ДАННЫХ В STEAMVR
        vr::VRBoneTransform_t boneTransforms[31];
        for (int i = 0; i < 31; i++) {
            if (i == 0) {
                boneTransforms[i].position.v[0] = worldX;
                boneTransforms[i].position.v[1] = worldY;
                boneTransforms[i].position.v[2] = worldZ;
            } else {
                boneTransforms[i].position.v[0] = 0.0f;
                boneTransforms[i].position.v[1] = 0.0f;
                boneTransforms[i].position.v[2] = 0.0f;
            }
            boneTransforms[i].orientation.x = finalQuat.x;
            boneTransforms[i].orientation.y = finalQuat.y;
            boneTransforms[i].orientation.z = finalQuat.z;
            boneTransforms[i].orientation.w = finalQuat.w;
        }

        vr::VRDriverInput()->UpdateSkeletonComponent(m_skeletonHandle, vr::VRSkeletalMotionRange_WithoutController, boneTransforms, 31);

        vr::DriverPose_t pose = GetPose();
        pose.vecPosition[0] = worldX;
        pose.vecPosition[1] = worldY;
        pose.vecPosition[2] = worldZ;
        pose.qRotation = { finalQuat.w, finalQuat.x, finalQuat.y, finalQuat.z };
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, pose, sizeof(vr::DriverPose_t));
    }
};

// --- МЕНЕДЖЕР ДРАЙВЕРА ---
class CServerDriver_PurpleHands : public vr::IServerTrackedDeviceProvider {
private:
    CHandDevice* m_pLeftHand;
    CHandDevice* m_pRightHand;
    SOCKET m_udpSocket;

public:
    virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

        m_pLeftHand = new CHandDevice(false);
        m_pRightHand = new CHandDevice(true);
        vr::VRServerDriverHost()->TrackedDeviceAdded("PurpleHand_Left_001", vr::TrackedDeviceClass_Controller, m_pLeftHand);
        vr::VRServerDriverHost()->TrackedDeviceAdded("PurpleHand_Right_001", vr::TrackedDeviceClass_Controller, m_pRightHand);

        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
        m_udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        
        sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(9001);
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        bind(m_udpSocket, (SOCKADDR*)&serverAddr, sizeof(serverAddr));

        u_long mode = 1;
        ioctlsocket(m_udpSocket, FIONBIO, &mode);

        return vr::VRInitError_None;
    }

    virtual void Cleanup() override {
        closesocket(m_udpSocket);
        WSACleanup();
        delete m_pLeftHand;
        delete m_pRightHand;
        m_pLeftHand = nullptr;
        m_pRightHand = nullptr;
    }

    virtual const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }

    // Вызывается SteamVR 144 раза в секунду
    virtual void RunFrame() override {
        HandPacket packet;
        sockaddr_in clientAddr;
        int clientLength = sizeof(clientAddr);
        
        // 1. Проверяем, пришли ли новые данные от Питона (30 раз в секунду)
        int bytesRead = recvfrom(m_udpSocket, (char*)&packet, sizeof(HandPacket), 0, (SOCKADDR*)&clientAddr, &clientLength);
        
        if (bytesRead == sizeof(HandPacket)) {
            if (packet.isRightHand == 1 && m_pRightHand) {
                m_pRightHand->OnPacketReceived(packet);
            } else if (packet.isRightHand == 0 && m_pLeftHand) {
                m_pLeftHand->OnPacketReceived(packet);
            }
        }

        // 2. Двигаем руки ВСЕГДА (144 раза в секунду), даже если новых данных не было
        if (m_pLeftHand) m_pLeftHand->ProcessFrame();
        if (m_pRightHand) m_pRightHand->ProcessFrame();
    }

    virtual bool ShouldBlockStandbyMode() override { return false; }
    virtual void EnterStandby() override {}
    virtual void LeaveStandby() override {}
};

CServerDriver_PurpleHands g_serverDriverPurpleHands;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#endif

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    if (strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName) == 0) {
        return &g_serverDriverPurpleHands;
    }
    if (pReturnCode) {
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    }
    return nullptr;
}