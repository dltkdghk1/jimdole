package com.ssafy.carrybot.config;

import jakarta.annotation.PostConstruct;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.context.annotation.Lazy;
import org.springframework.context.annotation.Primary;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.integration.channel.DirectChannel;
import org.springframework.integration.core.MessageProducer;
import org.springframework.integration.mqtt.core.DefaultMqttPahoClientFactory;
import org.springframework.integration.mqtt.core.MqttPahoClientFactory;
import org.springframework.integration.mqtt.inbound.MqttPahoMessageDrivenChannelAdapter;
import org.springframework.integration.mqtt.support.DefaultPahoMessageConverter;
import org.springframework.messaging.MessageChannel;
import org.springframework.messaging.MessageHandler;

import com.ssafy.carrybot.robot.handler.MqttRobotMessageHandler;

@Configuration
public class MqttConfig {

    @Value("${MQTT_BROKER}")
    private String brokerUrl;

    @Value("${MQTT_CLIENT_ID}")
    private String clientId;

    @Value("${MQTT_PUBLISHER_CLIENT_ID:${MQTT_CLIENT_ID}-publisher}")
    private String publisherClientId;
    //mqtt 처음 보낼 때 connection lost error로 시뮬에선 못 받음
    //.env에 추가함

    @Value("${MQTT_USERNAME}")
    private String username;

    @Value("${MQTT_PASSWORD}")
    private String password;

    private final MqttRobotMessageHandler messageHandler;

    private static final String[] SUBSCRIBE_TOPICS = {
            "robot/status",
            "robot/log",
            "robot/photo"
    };

    public MqttConfig(@Lazy MqttRobotMessageHandler messageHandler) {
        this.messageHandler = messageHandler;
    }

    // ✅ 발신용 MqttClient Bean 등록
    @Bean
    @Primary
    public MqttClient mqttPublisherClient() throws MqttException {
        MqttClient client = new MqttClient(brokerUrl, publisherClientId, null);

        MqttConnectOptions options = new MqttConnectOptions();
        options.setUserName(username);
        options.setPassword(password.toCharArray());
        options.setAutomaticReconnect(true);
        options.setCleanSession(false);
        options.setKeepAliveInterval(60);

        client.connect(options);
        return client;
    }

    // ✅ 수신용 MQTT Factory
    @Bean
    public MqttPahoClientFactory mqttClientFactory() {
        DefaultMqttPahoClientFactory factory = new DefaultMqttPahoClientFactory();
        MqttConnectOptions options = new MqttConnectOptions();
        options.setServerURIs(new String[]{brokerUrl});
        options.setUserName(username);
        options.setPassword(password.toCharArray());
        factory.setConnectionOptions(options);
        return factory;
    }

    @Bean
    public MessageChannel mqttInputChannel() {
        return new DirectChannel();
    }

//    @Bean
//    public MessageProducer inbound() {
//        MqttPahoMessageDrivenChannelAdapter adapter =
//                new MqttPahoMessageDrivenChannelAdapter(clientId, mqttClientFactory(), SUBSCRIBE_TOPICS);
//        adapter.setCompletionTimeout(5000);
//        adapter.setConverter(new DefaultPahoMessageConverter());
//        adapter.setQos(1);
//        adapter.setOutputChannel(mqttInputChannel());
//        return adapter;
//    }
@Bean
public MessageProducer inbound() {
    // 🔧 수신 전용 clientId 하드코딩 방식으로 분리
    String subscriberClientId = clientId + "-subscriber";

    MqttPahoMessageDrivenChannelAdapter adapter =
            new MqttPahoMessageDrivenChannelAdapter(subscriberClientId, mqttClientFactory(), SUBSCRIBE_TOPICS);

    adapter.setCompletionTimeout(5000);
    adapter.setConverter(new DefaultPahoMessageConverter());
    adapter.setQos(1);
    adapter.setOutputChannel(mqttInputChannel());

    return adapter;
}

    @Bean
    @ServiceActivator(inputChannel = "mqttInputChannel")
    public MessageHandler handler() {
        return message -> {
            String topic = (String) message.getHeaders().get("mqtt_receivedTopic");
            String payload = (String) message.getPayload();

            System.out.println("📥 수신된 MQTT 메시지: topic=" + topic + ", payload=" + payload);

            if (topic == null) {
                System.out.println("수신된 메시지에 topic이 없습니다!");
                return;
            }

            switch (topic) {
                case "robot/status" -> messageHandler.handleStatus(payload);
                case "robot/log" -> messageHandler.handleLog(payload);
                case "robot/photo" -> messageHandler.handlePhoto(payload);
                default -> System.out.println("📭 Unknown topic: " + topic);
            }
        };
    }

    @PostConstruct
    public void logMqttConfig() {
        System.out.println("📡 MQTT 설정 확인:");
        System.out.println("brokerUrl: " + brokerUrl);
        System.out.println("clientId: " + clientId);
        System.out.println("publisherClientId: " + publisherClientId);
        System.out.println("username: " + username);
        System.out.println("password: " + password);
        System.out.println("subscribed topics:");
        for (String t : SUBSCRIBE_TOPICS) {
            System.out.println("  ➤ " + t);
        }
    }
}
