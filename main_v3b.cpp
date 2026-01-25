/*
basic 2d ball physics sim
-separated physics and rendering part
-fixing substepping
-improving thresholding to improve physics stability when as rest
-adding onground state for balls
-adding box
-adding aabb collision (box - box collision)
-adding ball box collision
*/

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include <cmath>   // for std:: abs to use with float
#include <cstdlib> // for rand()
#include <ctime>   // for time()
#include <sstream> // for ostringstream

class Box
{
public:
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Color color;
    sf::Vector2f halfSize;
    float mass;
    bool onGround = false;

    Box(float x, float y, float w, float h) : position(x, y), halfSize(w / 2, h / 2)
    {
        color = sf::Color::Cyan;
        mass = w * h * 0.05f;
        float vx = static_cast<float>((std::rand() % 600) - 300);
        float vy = static_cast<float>((std::rand() % 600) - 300);
        velocity = {vx, vy};
    }
};

class Ball
{
public:
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Color color;
    float radius;
    float mass;
    bool onGround = false;

    Ball(float x, float y, float radius) : position(x, y), radius(radius)
    {

        color = sf::Color(std::rand() % 256, std::rand() % 256, std::rand() % 256);

        float vx = static_cast<float>((std::rand() % 600) - 300);
        float vy = static_cast<float>((std::rand() % 600) - 300);
        velocity = {vx, vy};

        mass = radius * radius;
    }
};

class World
{
public:
    std::vector<Ball> balls;
    std::vector<Box> boxes;
    sf::Vector2u bounds;

    const float GRAVITY = 1500.0f;
    const float DAMPING = 0.85f;
    const int SUB_STEP = 8; // number of times updating physics for each ball, every frame

    World(unsigned int width, unsigned int height)
    {
        bounds = {width, height};
    }

    void addBall(float x, float y, float radius)
    {
        balls.emplace_back(x, y, radius);
    }

    void addBox(float x, float y, float w, float h)
    {
        boxes.emplace_back(x, y, w, h);
    }

    void update(float dt)
    {

        float sub_dt = dt / static_cast<float>(SUB_STEP);

        for (int k = 0; k < SUB_STEP; k++)
        {
            for (auto &ball : balls)
            {
                ball.onGround = false;
            }
            for (auto &box : boxes)
            {
                box.onGround = false;
            }
            for (auto &ball : balls)
            {
                ball.velocity.y += GRAVITY * sub_dt;
                ball.position += ball.velocity * sub_dt;

                checkWalls(ball);
            }
            solveCollision();

            for (auto &box : boxes)
            {
                box.velocity.y += GRAVITY * sub_dt;
                box.position += box.velocity * sub_dt;
                checkWallsBox(box);
            }

            for (auto &ball : balls)
            {
                if (ball.onGround)
                {
                    ball.velocity.x *= 0.99f; // friction
                }
                else
                {
                    // ball.velocity.x *= 0.999f; // drag
                }
            }

            for (auto &box : boxes)
            {
                if (box.onGround)
                {
                    box.velocity.x *= 0.99f;
                }
            }
        }
    }

    void checkWallsBox(Box &box)
    {
        // left wall collision
        if (box.position.x - box.halfSize.x < 0)
        {
            box.position.x = box.halfSize.x;
            box.velocity.x = std::abs(box.velocity.x);
        }
        // right wall collision
        else if (box.position.x + box.halfSize.x > bounds.x)
        {
            box.position.x = bounds.x - box.halfSize.x;
            box.velocity.x = -std::abs(box.velocity.x);
        }

        // ceiling collision
        if (box.position.y - box.halfSize.y < 0)
        {
            box.position.y = box.halfSize.y;
            box.velocity.y = std::abs(box.velocity.y);
        }
        // floor collision
        else if (box.position.y + box.halfSize.y > bounds.y)
        {
            box.position.y = bounds.y - box.halfSize.y;
            if (box.velocity.y > 0)
            {
                box.velocity.y = -std::abs(box.velocity.y) * DAMPING;
            }

            if (std::abs(box.velocity.y) < 20.0f)
            {
                box.velocity.y = 0;
            }

            box.onGround = true;
        }
    }

    void checkWalls(Ball &ball)
    {
        // left right walls collision
        if (ball.position.x - ball.radius < 0)
        {
            ball.position.x = ball.radius;
            ball.velocity.x = std::abs(ball.velocity.x);
        }
        else if (ball.position.x + ball.radius > bounds.x)
        {
            ball.position.x = bounds.x - ball.radius;
            ball.velocity.x = -std::abs(ball.velocity.x);
        }

        // ceiling collision
        if (ball.position.y - ball.radius < 0)
        {
            ball.position.y = ball.radius;

            ball.velocity.y = std::abs(ball.velocity.y);
        }
        // floor collision
        else if (ball.position.y + ball.radius > bounds.y)
        {
            ball.position.y = bounds.y - ball.radius;
            // bounce only if moving down (prevent double damping)
            if (ball.velocity.y > 0)
            {
                ball.velocity.y = -std::abs(ball.velocity.y) * DAMPING;
            }
            // sleep threshold depending on gravity and substep
            if (std::abs(ball.velocity.y) < 20.0f)
            {
                ball.velocity.y = 0;
            }

            ball.onGround = true;
        }
    }

    void solveCollision()
    {
        const int SOLVER_ITERATION = 4;
        // multiple iterations to propogate ball onground
        for (int iter = 0; iter < SOLVER_ITERATION; iter++)
        {
            for (int i = 0; i < balls.size(); i++)
            {
                for (int j = i + 1; j < balls.size(); j++)
                {
                    handleCollision(balls[i], balls[j]);
                }
            }

            for (int i = 0; i < boxes.size(); i++)
            {
                for (int j = i + 1; j < boxes.size(); j++)
                {
                    handleCollisionBox(boxes[i], boxes[j]);
                }
            }

            for (auto &ball : balls)
            {
                for (auto &box : boxes)
                {
                    handleCollisionBallBox(ball, box);
                }
            }
        }
    }

    void handleCollisionBallBox(Ball &ball, Box &box)
    {
        sf::Vector2f ballRelPos = ball.position - box.position; // position of ball wrt to box

        // to find point on box closest to ball's center
        sf::Vector2f closestBoxPoint = ballRelPos;

        // clamping to find points on box
        if (closestBoxPoint.x > box.halfSize.x)
        {
            closestBoxPoint.x = box.halfSize.x;
        }
        else if (closestBoxPoint.x < -box.halfSize.x)
        {
            closestBoxPoint.x = -box.halfSize.x;
        }

        if (closestBoxPoint.y > box.halfSize.y)
        {
            closestBoxPoint.y = box.halfSize.y;
        }
        else if (closestBoxPoint.y < -box.halfSize.y)
        {
            closestBoxPoint.y = -box.halfSize.y;
        }

        sf::Vector2f delta = ballRelPos - closestBoxPoint;
        float distSq = delta.x * delta.x + delta.y * delta.y;

        // no collision
        if (distSq > ball.radius * ball.radius)
            return;

        // rare edge case where both are overlapping, we are dealing it by ignoring it
        //  to prevent div by 0 err. alternatively we could push but meh.
        if (distSq < 0.001f)
            return;

        float dist = sqrt(distSq);

        float overlap = ball.radius - dist;

        sf::Vector2f n = delta / dist; // normal vector from box to ball

        if (n.y < -0.7 && box.onGround)
        {
            ball.onGround = true;
        }
        if (n.y > 0.9 && ball.onGround)
        {
            box.onGround = true;
        }

        // separation (static resolution)
        float massSum = ball.mass + box.mass;
        ball.position = ball.position + n * overlap * (box.mass / massSum);
        box.position = box.position - n * overlap * (ball.mass / massSum);

        // impulse transfer (dynamic resolution)
        sf::Vector2f relVelBall = ball.velocity - box.velocity;
        float relVelAlongN = relVelBall.x * n.x + relVelBall.y * n.y;
        float e = 0.8f;
        float j = -(1.0f + e) * relVelAlongN;
        j /= (1.0f / ball.mass + 1.0f / box.mass);

        sf::Vector2f impulse = j * n; // scalar -> vector
        ball.velocity += impulse / ball.mass;
        box.velocity -= impulse / box.mass;
    }

    void handleCollisionBox(Box &boxA, Box &boxB)
    {
        sf::Vector2f posA = boxA.position;
        sf::Vector2f posB = boxB.position;

        sf::Vector2f delta = posA - posB;

        float overlapX = (boxA.halfSize.x + boxB.halfSize.x) - std::abs(boxA.position.x - boxB.position.x);
        float overlapY = (boxA.halfSize.y + boxB.halfSize.y) - std::abs(boxA.position.y - boxB.position.y);

        if (overlapX <= 0 || overlapY <= 0)
        {
            return;
        }

        float overlap;
        sf::Vector2f n; // normal at point of contact from b to a

        if (overlapX < overlapY)
        {
            overlap = overlapX;
            n = (delta.x < 0) ? sf::Vector2f(-1, 0) : sf::Vector2f(1, 0);
        }
        else
        {
            overlap = overlapY;
            n = (delta.y < 0) ? sf::Vector2f(0, -1) : sf::Vector2f(0, 1);
        }

        if (n.y < -0.7f && boxB.onGround)
        {
            boxA.onGround = true;
        }
        if (n.y > 0.7f && boxA.onGround)
        {
            boxB.onGround = true;
        }

        float massSum = boxA.mass + boxB.mass;

        boxA.position += n * overlap * (boxB.mass / massSum);
        boxB.position -= n * overlap * (boxA.mass / massSum);

        sf::Vector2f relVel = boxA.velocity - boxB.velocity;
        float velAlongNormal = relVel.x * n.x + relVel.y * n.y;

        if (velAlongNormal > 0)
            return;

        float e = 0.8f;
        if (std::abs(velAlongNormal) < 60.0f)
            e = 0.0f;

        float j = -(1 + e) * velAlongNormal;
        j /= (1.0f / boxA.mass) + (1.0f / boxB.mass);

        sf::Vector2f impulse = j * n;

        boxA.velocity += impulse / boxA.mass;
        boxB.velocity -= impulse / boxB.mass;
    }

    void handleCollision(Ball &ballA, Ball &ballB)
    {
        sf::Vector2f posA = ballA.position;
        sf::Vector2f posB = ballB.position;

        sf::Vector2f delta = posA - posB; // vector pointing B -> A

        float distSq = delta.x * delta.x + delta.y * delta.y;
        float minDist = ballA.radius + ballB.radius;

        // Collision!
        if (distSq < minDist * minDist)
        {
            float dist = std::sqrt(distSq);

            if (dist < 0.0001f)
                dist = 0.0001f; // prevent divide by zero if spawning at same place

            float overlap = minDist - dist;

            // Static resolution - to un-overlap balls

            sf::Vector2f n = delta / dist;

            // A on B n.y -> -ve & vice versa
            // 0.7 because it approxes 45 deg. A.B = |A||B|cos theta
            // cos 45 = 1/root2 = 0.707

            if (n.y < -0.7f && ballB.onGround)
            {
                ballA.onGround = true;
            }

            if (n.y > 0.7f && ballA.onGround)
            {
                ballB.onGround = true;
            }

            float mass_sum = ballA.mass + ballB.mass;

            ballA.position = (posA + n * overlap * (ballB.mass / mass_sum));
            ballB.position = (posB - n * overlap * (ballA.mass / mass_sum));

            // Dynamic resolution - to determine new velocity

            sf::Vector2f relVel = ballA.velocity - ballB.velocity;  // vel of a wrt b
            float velAlongNormal = relVel.x * n.x + relVel.y * n.y; // component vel of a wrt b along the normal

            if (velAlongNormal > 0)
                return; // means ball A is moving away from ball B

            /*
                dont want to deal with time so we use impulse instead of force
                J = del p = m. del v;
                F = ma;
                F = J/t;
                J = j.n since no friction impulse only along n
                --------------

                del v = J/m
                v_new - v_old = J/m
                v_new = v_old + J/m

                applying this to our case

                (dot n to get impulse along normal)

                v_new_a = v_old_a + j.n/m_a...1
                v_new_b = v_old_b - j.n/m_b...2

                restitution formula
                (v_new_a - v_new_b).n = -e.(v_old_a - v_old_b).n
                v_new_rel.n = -e.v_old_rel.n

                subtract 1-2
                v_new_rel = v_old_rel + j.n(1/m_a + 1/m_b)

                to get rel vel only along normal mul by n also n.n = 1

                v_new_rel.n = v_old_rel.n + j(1/m_a + 1/m_b)

                -e.v_old_rel.n = v_old_rel.n + j(1/m_a + 1/m_b)

                -(1+e)v_old_rel.n = j(1/m_a + 1/m_b)

                j = -(1+e)v_old_rel.n/(1/m_a + 1/m_b)
            */

            float e = 0.8;

            if (std::abs(velAlongNormal) < 60.0f)
            {
                e = 0;
            }

            float j = -(1 + e) * velAlongNormal;
            j /= (1.0f / ballA.mass) + (1.0f / ballB.mass);

            sf::Vector2f impulse = j * n;
            ballA.velocity = (ballA.velocity + impulse / ballA.mass);
            ballB.velocity = (ballB.velocity - impulse / ballB.mass);
        }
    }
};

void renderWorld(sf::RenderWindow &window, const World &world)
{
    sf::CircleShape circle;

    for (auto &ball : world.balls)
    {
        circle.setRadius(ball.radius);
        circle.setOrigin(ball.radius, ball.radius);
        circle.setPosition(ball.position);
        circle.setFillColor(ball.color);
        window.draw(circle);
    }

    sf::RectangleShape rect;

    for (auto &box : world.boxes)
    {
        rect.setOrigin({box.halfSize.x, box.halfSize.y});
        rect.setPosition(box.position);
        rect.setFillColor(box.color);
        rect.setSize({box.halfSize.x * 2, box.halfSize.y * 2});
        window.draw(rect);
    }
}

int main()
{
    const int WINDOW_WIDTH = 600;
    const int WINDOW_HEIGHT = 600;
    const int FRAME_RATE = 144;

    const std::string WINDOW_TITLE = "Ball Sim";
    std::ostringstream ss;

    // generating seed according to current time. the seed helps mixing up random so it is different every start of code
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), WINDOW_TITLE);
    window.setFramerateLimit(FRAME_RATE);
    // window.setFramerateLimit(0); // unlimited fps to for stress test

    World world(WINDOW_WIDTH, WINDOW_HEIGHT);
    world.addBall(10, 10, 5);

    sf::Clock clock;
    sf::Clock fpsClock;

    // fixed timestamp variable
    float accumulator = 0.0f;
    const float FIXED_TIMESTAMP = 1.0f / 60.0f; // 0.16s physics updates in interval of this time only

    while (window.isOpen())
    {
        sf::Event event;

        float dt = clock.restart().asSeconds();
        if (dt > 0.1f)
        {
            dt = 0.1f; // lag spike protection
        }
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed)
            {
                // objects.clear();

                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    float radius = static_cast<float>((std::rand() % 20) + 10);
                    world.addBall(event.mouseButton.x, event.mouseButton.y, radius);
                }
                else if (event.mouseButton.button == sf::Mouse::Right)
                {
                    world.addBox(event.mouseButton.x, event.mouseButton.y, 60, 40);
                }
            }
        }

        accumulator += dt;
        window.clear();
        while (accumulator >= FIXED_TIMESTAMP)
        {
            world.update(FIXED_TIMESTAMP);
            accumulator -= FIXED_TIMESTAMP;
        }

        renderWorld(window, world);

        window.display();

        if (fpsClock.getElapsedTime().asSeconds() > 0.1f)
        {
            float fps = 1.0 / dt;

            ss.str("");
            ss.clear();

            ss << "Balls: " << world.balls.size() << " | FPS: " << static_cast<int>(fps);
            window.setTitle(ss.str());

            fpsClock.restart();
        }
    }

    return 0;
}
