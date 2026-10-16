#include <unity.h>
#include "DriftAssist.h"
#include "DriftAssistJson.h"
#include <limits>

void setUp() {}
void tearDown() {}

static void threshold_and_manual_priority()
{
    DriftAssist d; DriftAssistSettings s;
    for (uint32_t t=4000;t<=1000000;t+=4000) TEST_ASSERT_EQUAL_FLOAT(600,d.update(600,100,120,0.004f,t,true,s));
    const float result=d.update(600,100,300,0.004f,1004000,true,s);
    TEST_ASSERT_TRUE(result<600 && result>=510);
    TEST_ASSERT_TRUE(d.update(200,100,300,0.004f,1008000,true,s)<=200);
    TEST_ASSERT_EQUAL_FLOAT(0,d.update(0,100,300,0.004f,1012000,true,s));
    TEST_ASSERT_EQUAL_FLOAT(0,d.reduction());
}
static void symmetry_and_no_accumulation()
{
    DriftAssist a,b; DriftAssistSettings s;
    float out=0;
    for(uint32_t t=4000;t<=2000000;t+=4000) {
        out=a.update(800,100,250,0.004f,t,true,s);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f,out,b.update(800,-100,-250,0.004f,t,true,s));
        TEST_ASSERT_TRUE(out>=679.99f && out<=800);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f,680,out);
    a.reset(); TEST_ASSERT_EQUAL_FLOAT(0,a.reduction());
}
static void interval_consistency_and_recovery()
{
    DriftAssist a,b; DriftAssistSettings s;
    float x=0,y=0;
    for(uint32_t t=4000;t<=1000000;t+=4000) x=a.update(1000,100,250,0.004f,t,true,s);
    for(uint32_t t=10000;t<=1000000;t+=10000) y=b.update(1000,100,250,0.01f,t,true,s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,x,y);
    const float before=a.reduction();
    x=a.update(1000,100,100,0.004f,1004000,false,s);
    TEST_ASSERT_TRUE(a.reduction()<before && a.reduction()>0);
    TEST_ASSERT_TRUE(x>850 && x<1000);
    for(uint32_t t=1008000;t<=6000000;t+=4000) x=a.update(1000,100,100,0.004f,t,false,s);
    TEST_ASSERT_EQUAL_FLOAT(1000,x);
}
static void reversal_and_timer_wrap()
{
    DriftAssist d; DriftAssistSettings s;
    uint32_t t=0xffff0000U;
    TEST_ASSERT_EQUAL_FLOAT(500,d.update(500,160,160,0.004f,t,true,s));
    for(unsigned i=1;i<63;++i) TEST_ASSERT_EQUAL_FLOAT(500,d.update(500,0,160,0.004f,t+i*4000U,true,s));
    TEST_ASSERT_TRUE(d.update(500,0,160,0.004f,t+252000U,true,s)<500);
    d.reset();
    for(unsigned i=0;i<300;++i) TEST_ASSERT_EQUAL_FLOAT(500,d.update(500,i%2 ? -160 : 160,160,0.004f,i*4000U,true,s));
}
static void configuration_and_mixer()
{
    DriftAssistSettings s, loaded; JsonDocument doc;
    driftSettingsToJson(doc.to<JsonObject>(),s);
    TEST_ASSERT_TRUE(driftSettingsFromJson(doc,loaded));
    TEST_ASSERT_EQUAL(8,loaded.channel);
    TEST_ASSERT_EQUAL_FLOAT(15,loaded.maxReductionPercent);
    for(const int value : {-1,4,17,264}) { doc["channel"]=value; TEST_ASSERT_FALSE(driftSettingsFromJson(doc,loaded)); }
    doc["channel"]=8; doc["attack_ms"]=65536+120; TEST_ASSERT_FALSE(driftSettingsFromJson(doc,loaded));
    doc["attack_ms"]=120; doc["threshold_dps"]="20"; TEST_ASSERT_FALSE(driftSettingsFromJson(doc,loaded));
    const float car[]={1,0,0,0,0,0,0,-1}; const bool servos[]={false,true};
    TEST_ASSERT_TRUE(driftMixerCompatible(car,servos,8));
    const float plane[]={1,1,0,0,0,0,0,1}; TEST_ASSERT_FALSE(driftMixerCompatible(plane,servos,8));
    s.maxReductionPercent=30; TEST_ASSERT_TRUE(driftSettingsValid(s));
    s.rampDps=std::numeric_limits<float>::infinity(); TEST_ASSERT_FALSE(driftSettingsValid(s));
}
static void configuration_save_guard()
{
    // Armed + connected is valid at zero throttle, including CH7 coexistence.
    // Explicit and automatic arming deliberately share the same rule.
    TEST_ASSERT_NULL(driftSettingsChangeError(true,true,true,true,988));
    for (const uint16_t throttleUs : {989,1000,1500,2012}) {
        TEST_ASSERT_EQUAL_STRING("Lower throttle to zero before changing drift assistance",
            driftSettingsChangeError(true,true,true,true,throttleUs));
    }
    TEST_ASSERT_NULL(driftSettingsChangeError(true,true,true,false,1500));
    TEST_ASSERT_NOT_NULL(driftSettingsChangeError(true,true,false,false,988));
    TEST_ASSERT_NOT_NULL(driftSettingsChangeError(true,true,false,true,0));
    TEST_ASSERT_NULL(driftSettingsChangeError(false,true,false,true,1500));
    TEST_ASSERT_NULL(driftSettingsChangeError(true,false,false,true,1500));
}
int main(int argc,char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(threshold_and_manual_priority);
    RUN_TEST(symmetry_and_no_accumulation);
    RUN_TEST(interval_consistency_and_recovery);
    RUN_TEST(reversal_and_timer_wrap);
    RUN_TEST(configuration_and_mixer);
    RUN_TEST(configuration_save_guard);
    return UNITY_END();
}
