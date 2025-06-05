# 1단계: Maven 빌드 (캐시를 위한 dependency prefetch 포함)
FROM maven:3.8.8-eclipse-temurin-17 AS build
WORKDIR /app

# ✅ 경로 수정
COPY pom.xml .
RUN mvn dependency:go-offline -B

# ✅ 소스 복사
COPY . .
RUN mvn clean package -DskipTests

# 2단계: 실행용
FROM openjdk:17
WORKDIR /app
COPY --from=build /app/target/*.jar app.jar

ENV SPRING_PROFILES_ACTIVE=prod
ENTRYPOINT ["java", "-jar", "app.jar"]
